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
 * ESP32UartBridgeDeck.c - Receive-only UART1 bridge for an ESP32 companion.
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
 * Hardware wiring (3.3 V logic levels only, the Crazyflie deck port is NOT
 * 5 V tolerant):
 *
 *   ESP32 TX  -> Crazyflie UART1 RX / RX1 (PC11)
 *   ESP32 RX  -> Crazyflie UART1 TX / TX1 (PC10), only if ever needed
 *   ESP32 GND -> Crazyflie GND            (common ground is mandatory)
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
#include "uart1.h"

/**
 * Baudrate of the link to the ESP32.
 *
 * Single point of change: the ESP32 firmware must use the same value.
 * Format is fixed at 8 data bits, no parity, 1 stop bit (8N1) and no flow
 * control, which is what uart1Init() configures.
 */
#define ESP32_UART_BRIDGE_BAUDRATE 115200

/** Task identity. Kept local so that no core header has to be touched. */
#define ESP32_UART_BRIDGE_TASK_NAME "ESP32UARTBR"
#define ESP32_UART_BRIDGE_TASK_STACKSIZE configMINIMAL_STACK_SIZE
#define ESP32_UART_BRIDGE_TASK_PRI 1

/**
 * Blocking timeout of a single character read.
 *
 * The task sleeps on the UART1 queue for this long, so an idle link costs no
 * CPU time at all. On expiry the task only drops a stale partial frame and
 * goes back to sleep.
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

static void esp32UartBridgeTask(void *param)
{
  uint8_t ch;

  (void)param;

  while (true) {
    if (uart1GetDataWithTimeout(&ch, ESP32_UART_BRIDGE_RX_TIMEOUT_TICKS)) {
      esp32UartBridgeParserFeedByte(&parser, (char)ch, &m);
    } else {
      /* Nothing received for a full timeout period. That is not an error (no
       * ESP32 may be attached at all), but a frame that was cut short by a
       * peer reset must not linger across the silence. A complete frame takes
       * ~1.1 ms at 115200 baud, so this never truncates a fragmented frame. */
      esp32UartBridgeParserResetFrame(&parser);
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
  uart1Init(ESP32_UART_BRIDGE_BAUDRATE);

  if (xTaskCreate(esp32UartBridgeTask, ESP32_UART_BRIDGE_TASK_NAME,
                  ESP32_UART_BRIDGE_TASK_STACKSIZE, NULL,
                  ESP32_UART_BRIDGE_TASK_PRI, NULL) != pdPASS) {
    DEBUG_PRINT("Failed to create the ESP32 UART bridge task\n");
    return;
  }

  DEBUG_PRINT("ESP32 UART bridge listening on UART1 at %d baud\n",
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
  .usedPeriph = DECK_USING_UART1,

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
LOG_GROUP_STOP(esp32Uart)
