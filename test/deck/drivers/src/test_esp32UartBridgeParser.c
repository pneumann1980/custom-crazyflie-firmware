// File under test: src/deck/drivers/src/esp32UartBridgeParser.c
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "unity.h"

#include "esp32UartBridgeParser.h"

static ESP32UartBridgeParser parser;
static ESP32UartBridgeData data;

void setUp(void)
{
  esp32UartBridgeParserInit(&parser);
  memset(&data, 0, sizeof(data));
}

void tearDown(void)
{
  // Empty
}

/** Feed a NUL terminated string, returns the number of frames applied. */
static uint32_t feed(const char *text)
{
  return esp32UartBridgeParserFeed(&parser, text, strlen(text), &data);
}

/** Feed a NUL terminated string one character per call. */
static uint32_t feedByteWise(const char *text)
{
  uint32_t frames = 0;

  for (size_t i = 0; i < strlen(text); i++) {
    if (esp32UartBridgeParserFeedByte(&parser, text[i], &data)) {
      frames++;
    }
  }

  return frames;
}

static void assertAllVarsAre(uint16_t expected)
{
  TEST_ASSERT_EQUAL_UINT16(expected, data.var1);
  TEST_ASSERT_EQUAL_UINT16(expected, data.var2);
  TEST_ASSERT_EQUAL_UINT16(expected, data.var3);
  TEST_ASSERT_EQUAL_UINT16(expected, data.var4);
}

// 1. Valid message for every id ------------------------------------------

void testFrameWithId1IsStoredInVar1(void)
{
  TEST_ASSERT_EQUAL_UINT32(1, feed("!0x01,0x3214#"));

  TEST_ASSERT_EQUAL_UINT16(0x3214, data.var1);
  TEST_ASSERT_EQUAL_UINT16(0, data.var2);
  TEST_ASSERT_EQUAL_UINT16(0, data.var3);
  TEST_ASSERT_EQUAL_UINT16(0, data.var4);
  TEST_ASSERT_EQUAL_UINT32(1, parser.stats.framesAccepted);
  TEST_ASSERT_EQUAL_UINT32(0, parser.stats.framesRejected);
}

void testFrameWithId2IsStoredInVar2(void)
{
  TEST_ASSERT_EQUAL_UINT32(1, feed("!0x02,0xbeef#"));

  TEST_ASSERT_EQUAL_UINT16(0, data.var1);
  TEST_ASSERT_EQUAL_UINT16(0xbeef, data.var2);
  TEST_ASSERT_EQUAL_UINT16(0, data.var3);
  TEST_ASSERT_EQUAL_UINT16(0, data.var4);
}

void testFrameWithId3IsStoredInVar3(void)
{
  TEST_ASSERT_EQUAL_UINT32(1, feed("!0x03,0x0042#"));

  TEST_ASSERT_EQUAL_UINT16(0, data.var1);
  TEST_ASSERT_EQUAL_UINT16(0, data.var2);
  TEST_ASSERT_EQUAL_UINT16(0x0042, data.var3);
  TEST_ASSERT_EQUAL_UINT16(0, data.var4);
}

void testFrameWithId4IsStoredInVar4(void)
{
  TEST_ASSERT_EQUAL_UINT32(1, feed("!0x04,0x12ab#"));

  TEST_ASSERT_EQUAL_UINT16(0, data.var1);
  TEST_ASSERT_EQUAL_UINT16(0, data.var2);
  TEST_ASSERT_EQUAL_UINT16(0, data.var3);
  TEST_ASSERT_EQUAL_UINT16(0x12ab, data.var4);
}

void testEveryIdIsRoutedToItsOwnVariable(void)
{
  TEST_ASSERT_EQUAL_UINT32(4, feed("!0x01,0x0001#!0x02,0x0002#"
                                   "!0x03,0x0003#!0x04,0x0004#"));

  TEST_ASSERT_EQUAL_UINT16(0x0001, data.var1);
  TEST_ASSERT_EQUAL_UINT16(0x0002, data.var2);
  TEST_ASSERT_EQUAL_UINT16(0x0003, data.var3);
  TEST_ASSERT_EQUAL_UINT16(0x0004, data.var4);
}

// 2. Value range limits ---------------------------------------------------

void testValueZeroIsAccepted(void)
{
  TEST_ASSERT_EQUAL_UINT32(1, feed("!0x01,0xFFFF#"));
  TEST_ASSERT_EQUAL_UINT32(1, feed("!0x01,0x0000#"));

  TEST_ASSERT_EQUAL_UINT16(0x0000, data.var1);
}

void testMaximumValueIsAccepted(void)
{
  TEST_ASSERT_EQUAL_UINT32(4, feed("!0x01,0xFFFF#!0x02,0xFFFF#"
                                   "!0x03,0xFFFF#!0x04,0xFFFF#"));

  assertAllVarsAre(0xFFFF);
}

// 3. Hexadecimal case -----------------------------------------------------

void testUpperAndLowerCaseHexDigitsDecodeIdentically(void)
{
  TEST_ASSERT_EQUAL_UINT32(1, feed("!0x01,0xabCD#"));
  TEST_ASSERT_EQUAL_UINT32(1, feed("!0x02,0xABcd#"));

  TEST_ASSERT_EQUAL_UINT16(0xABCD, data.var1);
  TEST_ASSERT_EQUAL_UINT16(0xABCD, data.var2);
}

void testUpperCaseIdDigitsAreAccepted(void)
{
  // 0x0A is not a mapped id, 0x04 written as "0X04" is.
  TEST_ASSERT_EQUAL_UINT32(1, feed("!0X04,0X00FF#"));

  TEST_ASSERT_EQUAL_UINT16(0x00FF, data.var4);
}

// 4. Unknown id -----------------------------------------------------------

void testUnknownIdLeavesAllVariablesUntouched(void)
{
  TEST_ASSERT_EQUAL_UINT32(1, feed("!0x01,0x1111#"));

  TEST_ASSERT_EQUAL_UINT32(0, feed("!0x00,0x2222#"));
  TEST_ASSERT_EQUAL_UINT32(0, feed("!0x05,0x3333#"));
  TEST_ASSERT_EQUAL_UINT32(0, feed("!0xff,0x4444#"));

  TEST_ASSERT_EQUAL_UINT16(0x1111, data.var1);
  TEST_ASSERT_EQUAL_UINT16(0, data.var2);
  TEST_ASSERT_EQUAL_UINT16(0, data.var3);
  TEST_ASSERT_EQUAL_UINT16(0, data.var4);
  TEST_ASSERT_EQUAL_UINT32(3, parser.stats.unknownIds);
  TEST_ASSERT_EQUAL_UINT32(0, parser.stats.framesRejected);
}

// 5. Missing start character ---------------------------------------------

void testFrameWithoutStartCharacterIsIgnored(void)
{
  TEST_ASSERT_EQUAL_UINT32(0, feed("0x01,0x3214#"));

  assertAllVarsAre(0);
  TEST_ASSERT_EQUAL_UINT32(0, parser.stats.framesAccepted);
  TEST_ASSERT_EQUAL_UINT32(0, parser.stats.framesRejected);
}

void testParserResynchronisesAfterGarbageWithoutStartCharacter(void)
{
  TEST_ASSERT_EQUAL_UINT32(1, feed("0x02,0x1234#\r\nnoise!0x01,0x3214#"));

  TEST_ASSERT_EQUAL_UINT16(0x3214, data.var1);
  TEST_ASSERT_EQUAL_UINT16(0, data.var2);
}

void testCarriageReturnAndNewlineBetweenFramesAreIgnored(void)
{
  TEST_ASSERT_EQUAL_UINT32(2, feed("\r\n!0x01,0x0001#\r\n!0x02,0x0002#\r\n"));

  TEST_ASSERT_EQUAL_UINT16(0x0001, data.var1);
  TEST_ASSERT_EQUAL_UINT16(0x0002, data.var2);
  TEST_ASSERT_EQUAL_UINT32(0, parser.stats.framesRejected);
}

// 6. Missing end character ------------------------------------------------

void testIncompleteFrameChangesNothing(void)
{
  TEST_ASSERT_EQUAL_UINT32(0, feed("!0x01,0x3214"));

  assertAllVarsAre(0);
  TEST_ASSERT_EQUAL_UINT32(0, parser.stats.framesAccepted);
}

void testFrameTerminatedByNewlineInsteadOfEndCharacterIsRejected(void)
{
  TEST_ASSERT_EQUAL_UINT32(0, feed("!0x01,0x3214\r\n!0x02,0x0002"));

  assertAllVarsAre(0);
}

// 7. Invalid hexadecimal digit -------------------------------------------

void testInvalidHexDigitInValueIsRejected(void)
{
  TEST_ASSERT_EQUAL_UINT32(0, feed("!0x01,0x3g14#"));

  assertAllVarsAre(0);
  TEST_ASSERT_EQUAL_UINT32(1, parser.stats.framesRejected);
}

void testInvalidHexDigitInIdIsRejected(void)
{
  TEST_ASSERT_EQUAL_UINT32(0, feed("!0xz1,0x3214#"));

  assertAllVarsAre(0);
  TEST_ASSERT_EQUAL_UINT32(1, parser.stats.framesRejected);
}

void testMissingHexPrefixIsRejected(void)
{
  TEST_ASSERT_EQUAL_UINT32(0, feed("!001,0x3214##"));
  TEST_ASSERT_EQUAL_UINT32(0, feed("!0x01,03214#"));

  assertAllVarsAre(0);
}

void testWrongSeparatorIsRejected(void)
{
  TEST_ASSERT_EQUAL_UINT32(0, feed("!0x01;0x3214#"));

  assertAllVarsAre(0);
  TEST_ASSERT_EQUAL_UINT32(1, parser.stats.framesRejected);
}

// 8. Value too short or too long -----------------------------------------

void testTooShortValueIsRejected(void)
{
  TEST_ASSERT_EQUAL_UINT32(0, feed("!0x01,0x321#"));

  assertAllVarsAre(0);
  TEST_ASSERT_EQUAL_UINT32(1, parser.stats.framesRejected);
}

void testTooLongValueIsRejected(void)
{
  TEST_ASSERT_EQUAL_UINT32(0, feed("!0x01,0x32145#"));

  assertAllVarsAre(0);
  TEST_ASSERT_EQUAL_UINT32(1, parser.stats.framesRejected);
}

void testTooShortIdIsRejected(void)
{
  TEST_ASSERT_EQUAL_UINT32(0, feed("!0x1,0x3214#"));

  assertAllVarsAre(0);
}

void testRejectedFrameDoesNotBlockTheFollowingValidFrame(void)
{
  TEST_ASSERT_EQUAL_UINT32(1, feed("!0x01,0x32145#!0x01,0x3214#"));

  TEST_ASSERT_EQUAL_UINT16(0x3214, data.var1);
  TEST_ASSERT_EQUAL_UINT32(1, parser.stats.framesRejected);
  TEST_ASSERT_EQUAL_UINT32(1, parser.stats.framesAccepted);
}

// 9. Buffer overflow ------------------------------------------------------

void testBufferOverflowDiscardsTheFrameAndResynchronises(void)
{
  // Far more body characters than ESP32_UART_BRIDGE_RX_BUFFER_LEN, no '#'.
  TEST_ASSERT_EQUAL_UINT32(0, feed("!0x01,0x321400000000000000000000000000"));

  assertAllVarsAre(0);
  TEST_ASSERT_EQUAL_UINT32(1, parser.stats.framesRejected);

  // The overflowing run is dropped, the next start character re-synchronises.
  TEST_ASSERT_EQUAL_UINT32(1, feed("!0x02,0x0007#"));
  TEST_ASSERT_EQUAL_UINT16(0x0007, data.var2);
}

void testOverflowIsCountedOnlyOncePerFrame(void)
{
  char noise[128];
  memset(noise, 'A', sizeof(noise) - 1);
  noise[0] = '!';
  noise[sizeof(noise) - 1] = '\0';

  TEST_ASSERT_EQUAL_UINT32(0, feed(noise));

  TEST_ASSERT_EQUAL_UINT32(1, parser.stats.framesRejected);
  assertAllVarsAre(0);
}

// 10. Restart on a new start character -----------------------------------

void testStartCharacterInsideIncompleteFrameStartsANewFrame(void)
{
  TEST_ASSERT_EQUAL_UINT32(1, feed("!0x01,0x32!0x02,0x0009#"));

  TEST_ASSERT_EQUAL_UINT16(0, data.var1);
  TEST_ASSERT_EQUAL_UINT16(0x0009, data.var2);
  TEST_ASSERT_EQUAL_UINT32(1, parser.stats.framesRejected);
  TEST_ASSERT_EQUAL_UINT32(1, parser.stats.framesAccepted);
}

void testRepeatedStartCharactersDoNotCorruptTheFollowingFrame(void)
{
  TEST_ASSERT_EQUAL_UINT32(1, feed("!!!0x03,0x00aa#"));

  TEST_ASSERT_EQUAL_UINT16(0x00aa, data.var3);
}

// 11. Back to back frames -------------------------------------------------

void testTwoImmediatelyConsecutiveFramesAreBothApplied(void)
{
  TEST_ASSERT_EQUAL_UINT32(2, feed("!0x01,0x0001#!0x02,0x0002#"));

  TEST_ASSERT_EQUAL_UINT16(0x0001, data.var1);
  TEST_ASSERT_EQUAL_UINT16(0x0002, data.var2);
  TEST_ASSERT_EQUAL_UINT32(2, parser.stats.framesAccepted);
  TEST_ASSERT_EQUAL_UINT32(0, parser.stats.framesRejected);
}

void testLastFrameOfARepeatedIdWins(void)
{
  TEST_ASSERT_EQUAL_UINT32(3, feed("!0x01,0x0001#!0x01,0x0002#!0x01,0x0003#"));

  TEST_ASSERT_EQUAL_UINT16(0x0003, data.var1);
}

// 12. Fragmented input ----------------------------------------------------

void testByteWiseFragmentedFrameIsApplied(void)
{
  TEST_ASSERT_EQUAL_UINT32(1, feedByteWise("!0x01,0x3214#"));

  TEST_ASSERT_EQUAL_UINT16(0x3214, data.var1);
}

void testFrameSplitAcrossReadsIsApplied(void)
{
  const char *stream = "!0x01,0x0001#!0x02,0xFFFF#!0x03,0x00ab#";
  const size_t length = strlen(stream);
  uint32_t frames = 0;

  // Split the stream at every possible position and feed the two halves as
  // two separate "UART reads".
  for (size_t split = 0; split <= length; split++) {
    esp32UartBridgeParserInit(&parser);
    memset(&data, 0, sizeof(data));

    frames = esp32UartBridgeParserFeed(&parser, stream, split, &data);
    frames += esp32UartBridgeParserFeed(&parser, &stream[split],
                                        length - split, &data);

    TEST_ASSERT_EQUAL_UINT32(3, frames);
    TEST_ASSERT_EQUAL_UINT16(0x0001, data.var1);
    TEST_ASSERT_EQUAL_UINT16(0xFFFF, data.var2);
    TEST_ASSERT_EQUAL_UINT16(0x00ab, data.var3);
  }
}

void testNoValueChangesBeforeTheEndCharacterArrives(void)
{
  const char *frame = "!0x01,0x3214#";

  for (size_t i = 0; i < strlen(frame) - 1; i++) {
    TEST_ASSERT_FALSE(esp32UartBridgeParserFeedByte(&parser, frame[i], &data));
    assertAllVarsAre(0);
  }

  TEST_ASSERT_TRUE(esp32UartBridgeParserFeedByte(&parser, '#', &data));
  TEST_ASSERT_EQUAL_UINT16(0x3214, data.var1);
}

// Housekeeping ------------------------------------------------------------

void testInitClearsStateAndStatistics(void)
{
  feed("!0x01,0x3214#!0x01,0xzz");

  esp32UartBridgeParserInit(&parser);

  TEST_ASSERT_EQUAL_UINT32(0, parser.stats.framesAccepted);
  TEST_ASSERT_EQUAL_UINT32(0, parser.stats.framesRejected);
  TEST_ASSERT_EQUAL_UINT32(0, parser.stats.unknownIds);

  // The partial frame from before init must not be completed afterwards.
  TEST_ASSERT_EQUAL_UINT32(0, feed("14#"));
  TEST_ASSERT_EQUAL_UINT16(0x3214, data.var1);
}

void testResetFrameDropsThePartialFrameButKeepsStatistics(void)
{
  feed("!0x01,0x3214#");
  feed("!0x02,0x00");

  esp32UartBridgeParserResetFrame(&parser);

  TEST_ASSERT_EQUAL_UINT32(0, feed("07#"));
  TEST_ASSERT_EQUAL_UINT16(0, data.var2);
  TEST_ASSERT_EQUAL_UINT32(1, parser.stats.framesAccepted);
}

void testDiscardPartialFrameCountsTheTruncatedFrame(void)
{
  feed("!0x01,0x3214#");
  feed("!0x02,0x00");

  TEST_ASSERT_TRUE(esp32UartBridgeParserDiscardPartialFrame(&parser));

  TEST_ASSERT_EQUAL_UINT32(1, parser.stats.framesAccepted);
  TEST_ASSERT_EQUAL_UINT32(1, parser.stats.framesRejected);
  TEST_ASSERT_EQUAL_UINT16(0, data.var2);

  // The remainder of the truncated frame must not complete it afterwards.
  TEST_ASSERT_EQUAL_UINT32(0, feed("07#"));
  TEST_ASSERT_EQUAL_UINT16(0, data.var2);
}

void testDiscardPartialFrameOutsideAFrameIsANoOp(void)
{
  TEST_ASSERT_EQUAL_UINT32(1, feed("!0x01,0x3214#"));

  TEST_ASSERT_FALSE(esp32UartBridgeParserDiscardPartialFrame(&parser));

  TEST_ASSERT_EQUAL_UINT32(1, parser.stats.framesAccepted);
  TEST_ASSERT_EQUAL_UINT32(0, parser.stats.framesRejected);
  TEST_ASSERT_EQUAL_UINT16(0x3214, data.var1);
}

void testDiscardPartialFrameIsIdempotent(void)
{
  feed("!0x03,0x1");

  TEST_ASSERT_TRUE(esp32UartBridgeParserDiscardPartialFrame(&parser));
  TEST_ASSERT_FALSE(esp32UartBridgeParserDiscardPartialFrame(&parser));

  TEST_ASSERT_EQUAL_UINT32(1, parser.stats.framesRejected);
}

void testFrameAfterDiscardIsStillParsed(void)
{
  feed("!0x04,0x12");
  esp32UartBridgeParserDiscardPartialFrame(&parser);

  TEST_ASSERT_EQUAL_UINT32(1, feed("!0x04,0x00ff#"));
  TEST_ASSERT_EQUAL_UINT16(0x00ff, data.var4);
}

void testEmptyFeedIsANoOp(void)
{
  TEST_ASSERT_EQUAL_UINT32(0, esp32UartBridgeParserFeed(&parser, NULL, 0, &data));

  assertAllVarsAre(0);
}
