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
 * esp32UartBridgeParser.h - Frame parser for the ESP32 UART bridge deck.
 *
 * The parser is deliberately kept free of any FreeRTOS, deck or logging
 * dependency so that it can be compiled and unit tested on the host.
 *
 * Wire format (ASCII, exactly 13 characters per frame):
 *
 *   !0xII,0xVVVV#
 *
 *   '!'    start of frame
 *   0xII   two-digit hexadecimal measurement id
 *   ','    separator
 *   0xVVVV four-digit hexadecimal uint16_t payload
 *   '#'    end of frame
 *
 * Hexadecimal digits may be upper or lower case. Any character received
 * outside of a frame (in particular '\r' and '\n') is silently discarded.
 */
#ifndef ESP32_UART_BRIDGE_PARSER_H_
#define ESP32_UART_BRIDGE_PARSER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** Number of characters of a complete, valid frame including '!' and '#'. */
#define ESP32_UART_BRIDGE_FRAME_LEN 13u

/** Number of characters between '!' and '#' of a valid frame. */
#define ESP32_UART_BRIDGE_BODY_LEN (ESP32_UART_BRIDGE_FRAME_LEN - 2u)

/**
 * Size of the static receive buffer holding the frame body.
 *
 * Slightly larger than ESP32_UART_BRIDGE_BODY_LEN so that an over-long frame
 * is rejected by the length check rather than by the overflow guard for the
 * common "one character too many" case. A frame body that grows beyond this
 * size trips the overflow guard, is discarded, and the parser resynchronises
 * on the next '!'.
 */
#define ESP32_UART_BRIDGE_RX_BUFFER_LEN 16u

/** Measurement ids understood by the deck. Unknown ids leave all data intact. */
#define ESP32_UART_BRIDGE_ID_VAR1 0x01u
#define ESP32_UART_BRIDGE_ID_VAR2 0x02u
#define ESP32_UART_BRIDGE_ID_VAR3 0x03u
#define ESP32_UART_BRIDGE_ID_VAR4 0x04u

/** Values published to the Crazyflie log framework. */
typedef struct {
  uint16_t var1;
  uint16_t var2;
  uint16_t var3;
  uint16_t var4;
} ESP32UartBridgeData;

/** Parser diagnostics, useful to judge link quality in the field. */
typedef struct {
  uint32_t framesAccepted;  //!< Frames that passed validation and were applied
  uint32_t framesRejected;  //!< Malformed frames, including buffer overflows
  uint32_t unknownIds;      //!< Well-formed frames carrying an unmapped id
} ESP32UartBridgeStats;

/** Receive states of the byte-wise state machine. */
typedef enum {
  ESP32_UART_BRIDGE_STATE_HUNT = 0,  //!< Outside a frame, waiting for '!'
  ESP32_UART_BRIDGE_STATE_BODY = 1,  //!< Inside a frame, collecting the body
} ESP32UartBridgeState;

/** Parser context. All storage is static/caller owned, nothing is allocated. */
typedef struct {
  ESP32UartBridgeState state;
  uint8_t length;
  char buffer[ESP32_UART_BRIDGE_RX_BUFFER_LEN];
  ESP32UartBridgeStats stats;
} ESP32UartBridgeParser;

/**
 * Reset the parser to the hunting state and clear the diagnostics counters.
 *
 * @param parser Parser context, must not be NULL.
 */
void esp32UartBridgeParserInit(ESP32UartBridgeParser *parser);

/**
 * Drop a partially received frame without touching the diagnostics counters.
 *
 * Used by the receive task to discard a stale partial frame after a long
 * period of silence on the link.
 *
 * @param parser Parser context, must not be NULL.
 */
void esp32UartBridgeParserResetFrame(ESP32UartBridgeParser *parser);

/**
 * Feed a single received character into the parser.
 *
 * @p data is modified only after a complete frame, terminated by '#', has
 * been fully validated and carries a known id.
 *
 * @param parser Parser context, must not be NULL.
 * @param c      Received character.
 * @param data   Destination for decoded measurements, must not be NULL.
 * @return true if this character completed a valid frame with a known id.
 */
bool esp32UartBridgeParserFeedByte(ESP32UartBridgeParser *parser, char c,
                                   ESP32UartBridgeData *data);

/**
 * Feed a block of received characters into the parser.
 *
 * @param parser Parser context, must not be NULL.
 * @param bytes  Received characters, may be NULL only when @p length is zero.
 * @param length Number of characters in @p bytes.
 * @param data   Destination for decoded measurements, must not be NULL.
 * @return Number of complete, valid frames applied to @p data.
 */
uint32_t esp32UartBridgeParserFeed(ESP32UartBridgeParser *parser,
                                   const char *bytes, size_t length,
                                   ESP32UartBridgeData *data);

#endif /* ESP32_UART_BRIDGE_PARSER_H_ */
