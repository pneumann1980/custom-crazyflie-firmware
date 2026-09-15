/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2026 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * ESP32UartBridgeDeck.c - Receive-only UART bridge for an ESP32 companion.
 *
 * The ESP32 pushes ASCII framed measurements at us:
 *
 *   !0xII,0xVVVV#
 *
 * Four ids are mapped onto four uint16_t log variables (esp32Uart.var1 ..
 * esp32Uart.var4). The driver never transmits: it sends no configuration
 * command to the ESP32 and does not mirror received characters to the
 * console.
 *
 * The port is selected at build time, because UART1 clashes with decks that
 * use RX1/TX1 as plain GPIO - the Loco deck does exactly that unless
 * CONFIG_DECK_LOCODECK_USE_ALT_PINS is set.
 *
 * Hardware wiring (3.3 V logic levels only, the Crazyflie deck port is NOT
 * 5 V tolerant):
 *
 *   CONFIG_DECK_ESP32_UART_BRIDGE_ON_UART1 (default)
 *     ESP32 TX  -> Crazyflie RX1 (PC11)
 *     ESP32 RX  -> Crazyflie TX1 (PC10), only if ever needed
 *
 *   CONFIG_DECK_ESP32_UART_BRIDGE_ON_UART2
 *     ESP32 TX  -> Crazyflie RX2 (PA3)
 *     ESP32 RX  -> Crazyflie TX2 (PA2), only if ever needed
 *
 *   ESP32 GND -> Crazyflie GND, in both cases (common ground is mandatory)
 *
 * The ESP32 must send plain ASCII with leading zeroes, e.g. printf("!0x%02X,"
 * "0x%04X#", id, value). A binary frame of '!', 0x01, two raw value bytes and
 * '#' is NOT understood.
 *
 * This driver is meant to be loaded through CONFIG_DECK_FORCE, so it must
 * come up cleanly with no ESP32 attached: initialisation never waits for the
 * peer, the receive task only blocks on the UART queue, and the log variables
 * simply stay at 0 while the link is silent.
 */
#define DEBUG_MODULE "ESP32UART"

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "deck.h"
#include "debug.h"
#include "esp32UartBridgeParser.h"
#include "log.h"
#include "param.h"

#include "autoconf.h"

/*
 * Port abstraction. Both drivers expose the same "read one byte, block up to
 * n ticks" primitive, so the rest of the file is port agnostic.
 */
#ifdef CONFIG_DECK_ESP32_UART_BRIDGE_ON_UART2
  #include "uart2.h"
  #define BRIDGE_UART_NAME            "UART2"
  #define BRIDGE_UART_PERIPH          DECK_USING_UART2
  #define BRIDGE_UART_INIT(baudrate)  uart2Init(baudrate)
  #define BRIDGE_UART_GETCHAR(c, t)   uart2GetCharWithTimeout((c), (t))
  #define BRIDGE_UART_DID_OVERRUN()   uart2DidOverrun()
#else
  #include "uart1.h"
  #define BRIDGE_UART_NAME            "UART1"
  #define BRIDGE_UART_PERIPH          DECK_USING_UART1
  #define BRIDGE_UART_INIT(baudrate)  uart1Init(baudrate)
  #define BRIDGE_UART_GETCHAR(c, t)   uart1GetDataWithTimeout((c), (t))
  #define BRIDGE_UART_DID_OVERRUN()   uart1DidOverrun()
#endif

/**
 * Baudrate of the link to the ESP32.
 *
 * Single point of change: the ESP32 firmware must use the same value.
 * Format is fixed at 8 data bits, no parity, 1 stop bit (8N1) and no flow
 * control, which is what the uartXInit() of the selected port configures.
 */
#define ESP32_UART_BRIDGE_BAUDRATE 115200

/**
 * Task identity. Kept local so that no core header has to be touched.
 *
 * configMAX_TASK_NAME_LEN is 10 including the terminator, so at most 9
 * characters survive into the FreeRTOS task list - a longer name would only
 * show up truncated in the task dump.
 */
#define ESP32_UART_BRIDGE_TASK_NAME "ESP32UART"
#define ESP32_UART_BRIDGE_TASK_STACKSIZE configMINIMAL_STACK_SIZE
#define ESP32_UART_BRIDGE_TASK_PRI 1

/**
 * Blocking timeout of a single character read.
 *
 * The task sleeps on the receive queue of the selected port for this long, so
 * an idle link costs no CPU time at all. On expiry the task only drops a stale
 * partial frame and goes back to sleep.
 */
#define ESP32_UART_BRIDGE_RX_TIMEOUT_MS 1000
#define ESP32_UART_BRIDGE_RX_TIMEOUT_TICKS M2T(ESP32_UART_BRIDGE_RX_TIMEOUT_MS)

static bool isInit = false;

/**
 * Values exported to the log framework, all zero until data arrives.
 *
 * Written by the receive task, read by the log task. No lock is needed: each
 * member is a naturally aligned 16-bit word, so a single value is never read
 * half updated on Cortex-M4. A log block covering several members can still
 * straddle two frames, which is inherent to the protocol - one frame only ever
 * carries one measurement.
 */
static ESP32UartBridgeData m;

static ESP32UartBridgeParser parser;

/** UART receive overruns seen since boot, i.e. bytes lost before the parser. */
static uint32_t uartOverruns;

/** Tick of the last accepted frame, only valid once hasReceivedFrame is set. */
static uint32_t lastFrameTick;
static bool hasReceivedFrame;

/** Value reported by esp32Uart.age while no valid frame has arrived yet. */
#define ESP32_UART_BRIDGE_AGE_NEVER UINT32_MAX

static void esp32UartBridgeTask(void *param)
{
  uint8_t ch;

  (void)param;

  while (true) {
    if (BRIDGE_UART_GETCHAR(&ch, ESP32_UART_BRIDGE_RX_TIMEOUT_TICKS)) {
      if (esp32UartBridgeParserFeedByte(&parser, (char)ch, &m)) {
        lastFrameTick = xTaskGetTickCount();
        hasReceivedFrame = true;
      }
    } else {
      /* Nothing received for a full timeout period. That is not an error (no
       * ESP32 may be attached at all), but a frame that was cut short by a
       * peer reset must not linger across the silence. A complete frame takes
       * ~1.1 ms at 115200 baud, so this never truncates a fragmented frame.
       * The partial frame is counted as rejected, which is what esp32Uart.nErr
       * promises. */
      esp32UartBridgeParserDiscardPartialFrame(&parser);
    }

    /* Sticky flag, cleared by the read. Bytes lost here never reach the
     * parser, so nErr would stay silent about them. */
    if (BRIDGE_UART_DID_OVERRUN()) {
      uartOverruns++;
    }
  }
}

static void esp32UartBridgeInit(DeckInfo *info)
{
  (void)info;

  if (isInit) {
    return;
  }

  esp32UartBridgeParserInit(&parser);

  /* Configures the peripheral and the RX queue only, it does not wait. */
  BRIDGE_UART_INIT(ESP32_UART_BRIDGE_BAUDRATE);

#if defined(CONFIG_DECK_LOCO) && \
    !defined(CONFIG_DECK_ESP32_UART_BRIDGE_ON_UART2) && \
    !defined(CONFIG_DECK_LOCODECK_USE_ALT_PINS)
  /* The Loco deck drives RX1/TX1 as IRQ and RESET GPIOs. It declares them in
   * usedGpio while this driver declares them in usedPeriph, and
   * checkPeriphAndGpioConflicts() never compares those two fields against each
   * other - so the clash is NOT caught and both drivers would configure PC10
   * and PC11. Warn loudly; the fix is either alternative Loco pins or moving
   * this deck to UART2. */
  DEBUG_PRINT("WARNING: Loco deck support is built in and uses RX1/TX1. "
              "If a Loco deck is attached, set CONFIG_DECK_LOCODECK_USE_ALT_PINS "
              "or move this deck to UART2.\n");
#endif

  if (xTaskCreate(esp32UartBridgeTask, ESP32_UART_BRIDGE_TASK_NAME,
                  ESP32_UART_BRIDGE_TASK_STACKSIZE, NULL,
                  ESP32_UART_BRIDGE_TASK_PRI, NULL) != pdPASS) {
    DEBUG_PRINT("Failed to create the ESP32 UART bridge task\n");
    return;
  }

  DEBUG_PRINT("ESP32 UART bridge listening on " BRIDGE_UART_NAME " at %d baud\n",
              ESP32_UART_BRIDGE_BAUDRATE);

  isInit = true;
}

/**
 * Report whether the driver came up.
 *
 * Deliberately does not look at received data: with CONFIG_DECK_FORCE the
 * deck may well be absent, and a silent link must not fail the self test and
 * block arming.
 */
static bool esp32UartBridgeTest(void)
{
  return isInit;
}

static const DeckDriver esp32UartBridge_deck = {
  /* Development deck loaded through CONFIG_DECK_FORCE only. It carries no
   * OW memory, so the ids are never matched against real hardware; 0x00/0x00
   * keeps them clear of the Bitcraze vid 0xBC namespace. */
  .vid = 0x00,
  .pid = 0x00,
  .name = "esp32UartBridge",

  .usedGpio = 0,
  .usedPeriph = BRIDGE_UART_PERIPH,

  .init = esp32UartBridgeInit,
  .test = esp32UartBridgeTest,
};

DECK_DRIVER(esp32UartBridge_deck);

PARAM_GROUP_START(deck)
/**
 * @brief Nonzero if the ESP32 UART bridge deck driver has been initialised
 */
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, esp32UartBridge, &isInit)
PARAM_GROUP_STOP(deck)

/**
 * Age of the newest measurement, evaluated when the ground station reads it.
 *
 * Computed on read rather than stored, so it stays accurate while the receive
 * task sits blocked on the UART queue.
 */
static uint32_t ageLogger(uint32_t timestamp, void *data)
{
  (void)timestamp;
  (void)data;

  if (!hasReceivedFrame) {
    return ESP32_UART_BRIDGE_AGE_NEVER;
  }

  /* Unsigned tick subtraction stays correct across the 32 bit wrap. */
  return (uint32_t)T2M(xTaskGetTickCount() - lastFrameTick);
}

static logByFunction_t ageLoggerDef = {.acquireUInt32 = ageLogger, .data = 0};

LOG_GROUP_START(esp32Uart)
/**
 * @brief Measurement received with id 0x01
 */
LOG_ADD(LOG_UINT16, var1, &m.var1)
/**
 * @brief Measurement received with id 0x02
 */
LOG_ADD(LOG_UINT16, var2, &m.var2)
/**
 * @brief Measurement received with id 0x03
 */
LOG_ADD(LOG_UINT16, var3, &m.var3)
/**
 * @brief Measurement received with id 0x04
 */
LOG_ADD(LOG_UINT16, var4, &m.var4)
/**
 * @brief Number of valid frames applied since boot
 */
LOG_ADD(LOG_UINT32, nOk, &parser.stats.framesAccepted)
/**
 * @brief Number of malformed or truncated frames discarded since boot
 */
LOG_ADD(LOG_UINT32, nErr, &parser.stats.framesRejected)
/**
 * @brief Number of valid frames carrying an unmapped id since boot
 */
LOG_ADD(LOG_UINT32, nUnknown, &parser.stats.unknownIds)
/**
 * @brief Number of UART receive overruns since boot, i.e. bytes lost in the
 *        driver before they could reach the parser
 */
LOG_ADD(LOG_UINT32, nOvr, &uartOverruns)
/**
 * @brief Milliseconds since the last valid frame, UINT32_MAX while none has
 *        been received. Lets the ground station tell a fresh value from a
 *        stale one that is merely still in the log variable.
 */
LOG_ADD_BY_FUNCTION(LOG_UINT32, age, &ageLoggerDef)
LOG_GROUP_STOP(esp32Uart)
