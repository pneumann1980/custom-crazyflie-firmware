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
 * esp32UartBridgeParser.c - Frame parser for the ESP32 UART bridge deck.
 *
 * No dynamic allocation, no unbounded string operations and no dependency on
 * the RTOS: everything runs out of the caller owned ESP32UartBridgeParser.
 */

#include <string.h>

#include "esp32UartBridgeParser.h"

#define FRAME_START_CHAR '!'
#define FRAME_END_CHAR   '#'
#define FRAME_SEPARATOR  ','

/* Byte offsets inside the collected frame body "0xII,0xVVVV". */
#define BODY_ID_PREFIX_POS  0u
#define BODY_ID_DIGITS_POS  2u
#define BODY_SEPARATOR_POS  4u
#define BODY_VAL_PREFIX_POS 5u
#define BODY_VAL_DIGITS_POS 7u

#define ID_DIGITS    2u
#define VALUE_DIGITS 4u

/**
 * Convert a single hexadecimal character.
 *
 * @param c   Character to convert.
 * @param out Receives the value 0..15 on success, untouched otherwise.
 * @return true if @p c is a valid hexadecimal digit.
 */
static bool hexDigitValue(char c, uint8_t *out)
{
  if (c >= '0' && c <= '9') {
    *out = (uint8_t)(c - '0');
    return true;
  }
  if (c >= 'a' && c <= 'f') {
    *out = (uint8_t)(c - 'a' + 10);
    return true;
  }
  if (c >= 'A' && c <= 'F') {
    *out = (uint8_t)(c - 'A' + 10);
    return true;
  }
  return false;
}

/**
 * Convert a fixed number of hexadecimal characters into an unsigned value.
 *
 * Deliberately used instead of sscanf()/strtoul(): the conversion is bounded
 * by @p digits, needs no NUL terminated input and pulls no formatted-input
 * machinery into the image.
 *
 * @param chars  First character of the digit run.
 * @param digits Number of digits to convert, at most 4.
 * @param out    Receives the converted value on success.
 * @return true if every converted character was a valid hexadecimal digit.
 */
static bool hexToUint16(const char *chars, uint8_t digits, uint16_t *out)
{
  uint16_t value = 0;

  for (uint8_t i = 0; i < digits; i++) {
    uint8_t nibble;
    if (!hexDigitValue(chars[i], &nibble)) {
      return false;
    }
    value = (uint16_t)((value << 4) | nibble);
  }

  *out = value;
  return true;
}

/** Accept both "0x" and "0X" as the numeric prefix of a field. */
static bool hasHexPrefix(const char *chars)
{
  return chars[0] == '0' && (chars[1] == 'x' || chars[1] == 'X');
}

/**
 * Validate the collected frame body and decode id and value from it.
 *
 * @param parser Parser holding the body collected so far.
 * @param id     Receives the decoded id on success.
 * @param value  Receives the decoded value on success.
 * @return true if the body matches "0xII,0xVVVV" exactly.
 */
static bool decodeBody(const ESP32UartBridgeParser *parser, uint8_t *id,
                       uint16_t *value)
{
  uint16_t rawId;

  if (parser->length != ESP32_UART_BRIDGE_BODY_LEN) {
    return false;
  }
  if (!hasHexPrefix(&parser->buffer[BODY_ID_PREFIX_POS])) {
    return false;
  }
  if (parser->buffer[BODY_SEPARATOR_POS] != FRAME_SEPARATOR) {
    return false;
  }
  if (!hasHexPrefix(&parser->buffer[BODY_VAL_PREFIX_POS])) {
    return false;
  }
  if (!hexToUint16(&parser->buffer[BODY_ID_DIGITS_POS], ID_DIGITS, &rawId)) {
    return false;
  }
  if (!hexToUint16(&parser->buffer[BODY_VAL_DIGITS_POS], VALUE_DIGITS, value)) {
    return false;
  }

  /* Two hexadecimal digits never exceed 0xFF, so the cast is lossless. */
  *id = (uint8_t)rawId;
  return true;
}

/**
 * Store a decoded measurement.
 *
 * @return true if the id is mapped, false if it is unknown. Unknown ids leave
 *         @p data completely untouched.
 */
static bool applyMeasurement(uint8_t id, uint16_t value,
                             ESP32UartBridgeData *data)
{
  switch (id) {
    case ESP32_UART_BRIDGE_ID_VAR1:
      data->var1 = value;
      return true;
    case ESP32_UART_BRIDGE_ID_VAR2:
      data->var2 = value;
      return true;
    case ESP32_UART_BRIDGE_ID_VAR3:
      data->var3 = value;
      return true;
    case ESP32_UART_BRIDGE_ID_VAR4:
      data->var4 = value;
      return true;
    default:
      return false;
  }
}

void esp32UartBridgeParserResetFrame(ESP32UartBridgeParser *parser)
{
  parser->state = ESP32_UART_BRIDGE_STATE_HUNT;
  parser->length = 0;
}

void esp32UartBridgeParserInit(ESP32UartBridgeParser *parser)
{
  memset(parser, 0, sizeof(*parser));
  esp32UartBridgeParserResetFrame(parser);
}

bool esp32UartBridgeParserFeedByte(ESP32UartBridgeParser *parser, char c,
                                   ESP32UartBridgeData *data)
{
  uint8_t id;
  uint16_t value;

  /* A start character always begins a new frame, discarding any partial one. */
  if (c == FRAME_START_CHAR) {
    if (parser->state == ESP32_UART_BRIDGE_STATE_BODY) {
      parser->stats.framesRejected++;
    }
    parser->state = ESP32_UART_BRIDGE_STATE_BODY;
    parser->length = 0;
    return false;
  }

  /* Outside a frame everything, '\r' and '\n' included, is discarded. */
  if (parser->state != ESP32_UART_BRIDGE_STATE_BODY) {
    return false;
  }

  if (c != FRAME_END_CHAR) {
    if (parser->length >= ESP32_UART_BRIDGE_RX_BUFFER_LEN) {
      /* Overflow: drop the frame and resynchronise on the next '!'. */
      parser->stats.framesRejected++;
      esp32UartBridgeParserResetFrame(parser);
      return false;
    }
    parser->buffer[parser->length++] = c;
    return false;
  }

  /* End of frame: validate the collected body before touching the parser
   * state, then resynchronise regardless of the outcome. */
  const bool bodyIsValid = decodeBody(parser, &id, &value);
  esp32UartBridgeParserResetFrame(parser);

  if (!bodyIsValid) {
    parser->stats.framesRejected++;
    return false;
  }
  if (!applyMeasurement(id, value, data)) {
    parser->stats.unknownIds++;
    return false;
  }

  parser->stats.framesAccepted++;
  return true;
}

uint32_t esp32UartBridgeParserFeed(ESP32UartBridgeParser *parser,
                                   const char *bytes, size_t length,
                                   ESP32UartBridgeData *data)
{
  uint32_t frames = 0;

  for (size_t i = 0; i < length; i++) {
    if (esp32UartBridgeParserFeedByte(parser, bytes[i], data)) {
      frames++;
    }
  }

  return frames;
}
