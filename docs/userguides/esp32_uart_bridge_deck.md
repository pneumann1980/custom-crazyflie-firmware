---
title: ESP32 UART bridge deck
page_id: esp32_uart_bridge_deck
---

Receive-only deck driver that reads ASCII framed measurements from an ESP32 on
UART1 and publishes four of them as log variables. It is a development deck
without OW memory, so it is loaded through `CONFIG_DECK_FORCE`.

| Item | Value |
| --- | --- |
| Driver | `src/deck/drivers/src/ESP32UartBridgeDeck.c` |
| Parser | `src/deck/drivers/src/esp32UartBridgeParser.c` |
| Deck name | `esp32UartBridge` |
| Kconfig | `CONFIG_DECK_ESP32_UART_BRIDGE` |
| Port | UART1 (default) or UART2, 115200 baud, 8N1, no flow control |
| Log group | `esp32Uart` |

## Choosing the UART port

The port is a build-time choice, because UART1 collides with decks that use
RX1/TX1 as plain GPIO.

| Deck combination | Port to use | Config |
| --- | --- | --- |
| Bridge alone | UART1 | `esp32_uart_bridge_defconfig` |
| Bridge + Flow | UART1 | `esp32_uart_bridge_defconfig` |
| Bridge + Loco (default Loco pins) | **UART2** | `esp32_uart_bridge_uart2_defconfig` |
| Bridge + Loco + Flow | **UART2** | `esp32_uart_bridge_uart2_defconfig` |
| Bridge + AI deck | UART1 | AI deck owns UART2 (CPX) |

Pin usage of the relevant drivers:

| Deck | Pins |
| --- | --- |
| Loco, default | `IO_1` (CS), `RX1`+`TX1` (IRQ, RESET), SPI |
| Loco, `CONFIG_DECK_LOCODECK_USE_ALT_PINS` | `IO_1`, `IO_2` (IRQ), `IO_3` (RESET), SPI |
| Loco, additionally `CONFIG_LOCODECK_ALT_PIN_RESET` | `IO_1`, `IO_2`, `IO_4` (RESET), SPI — **needs a hardware-modified Loco deck** |
| Flow v1/v2 | `IO_3`, I2C, SPI |
| AI deck | `IO_1`, `IO_4`, UART2 |
| This bridge | UART1 (`PC10`/`PC11`) or UART2 (`PA2`/`PA3`) |

On the Crazyflie 2.1 Brushless the motors sit on `PA1`, `PB11`, `PA15` and
`PB10` (`motorMapCF21Brushless`), so `PA2`/`PA3` are free. A BigQuad deck does
drive the motors from `PA2`/`PA3` and is therefore incompatible with the UART2
variant.

### Why the Loco clash is not caught automatically

`checkPeriphAndGpioConflicts()` compares `usedPeriph` against `usedPeriph` and
`usedGpio` against `usedGpio`, but never one against the other. The Loco deck
declares `PC10`/`PC11` in `usedGpio`, this deck declares them in `usedPeriph`
(as `DECK_USING_UART1`), so the overlap is invisible to that check and both
drivers would configure the same pins without any error message. The driver
therefore prints a warning at init when the Loco driver is built in without
alternative pins. A UART2-vs-UART2 clash with the AI deck *is* detected, since
both declare it in `usedPeriph`.

## Wiring

The Crazyflie deck port is 3.3 V only and is **not** 5 V tolerant. Use an ESP32
board whose UART pins run at 3.3 V, or insert a level shifter.

UART1 variant:

```text
ESP32 TX  -> Crazyflie RX1 (PC11)
ESP32 RX  -> Crazyflie TX1 (PC10), only if ever needed
ESP32 GND -> Crazyflie GND
```

UART2 variant:

```text
ESP32 TX  -> Crazyflie RX2 (PA3)
ESP32 RX  -> Crazyflie TX2 (PA2), only if ever needed
ESP32 GND -> Crazyflie GND
```

The driver never transmits, so `ESP32 RX` may be left unconnected. A common
ground is mandatory: without it the receiver sees floating levels and the
parser will reject everything it collects.

## Message format

```text
!0xII,0xVVVV#
```

* `!` start of frame
* `II` two hexadecimal digits, the measurement id
* `,` separator
* `VVVV` four hexadecimal digits, the `uint16_t` payload
* `#` end of frame

The frame is **13 ASCII characters**, not a binary packet. The sender must
format with leading zeroes, e.g. `printf("!0x%02X,0x%04X#", id, value)`. A
frame consisting of `!`, the raw byte `0x01`, two raw value bytes and `#` is
rejected.

Hexadecimal digits may be upper or lower case; `0X` is accepted as well as
`0x`. `\r` and `\n` between frames are ignored. Frames may follow each other
without a separator:

```text
!0x01,0x0001#!0x02,0x0002#
```

| id | log variable |
| --- | --- |
| `0x01` | `esp32Uart.var1` |
| `0x02` | `esp32Uart.var2` |
| `0x03` | `esp32Uart.var3` |
| `0x04` | `esp32Uart.var4` |

Unknown ids are counted but change no variable. A target variable is written
only after the complete frame, `#` included, has been validated.

## Log variables

| Name | Type | Meaning |
| --- | --- | --- |
| `esp32Uart.var1` .. `esp32Uart.var4` | `uint16_t` | Last received measurement per id, `0` until the first valid frame |
| `esp32Uart.nOk` | `uint32_t` | Valid frames applied since boot |
| `esp32Uart.nErr` | `uint32_t` | Malformed, truncated or overflowing frames discarded since boot |
| `esp32Uart.nUnknown` | `uint32_t` | Valid frames with an unmapped id |
| `esp32Uart.nOvr` | `uint32_t` | UART receive overruns, i.e. bytes lost before the parser saw them |
| `esp32Uart.age` | `uint32_t` | Milliseconds since the last valid frame, `0xFFFFFFFF` while none has arrived |

The counters are not part of the requested interface, but they separate the
failure modes quickly:

| Symptom | Likely cause |
| --- | --- |
| `nOk` = 0, `nErr` = 0, `age` = `0xFFFFFFFF` | Nothing received at all: wiring, ground, or ESP32 not sending |
| `nErr` rising, `nOk` = 0 | Wrong baudrate, inverted levels, or binary instead of ASCII framing |
| `nUnknown` rising | Sender uses ids outside `0x01`..`0x04` |
| `nOvr` rising | Data rate too high for the task to keep up, or long interrupt blocking |
| `nOk` stalled, `age` growing | Link died after working; values in `var1`..`var4` are stale |

**The four values are four independent "last seen" values.** A single log row
does not guarantee that all four come from the same ESP32 measurement cycle.
If they form one coherent vector, either send all four in one frame or add a
shared sequence number to the protocol.

## Build

```bash
# UART1 (RX1/TX1) - no Loco deck on default pins
make esp32_uart_bridge_defconfig
make -j"$(nproc)"

# UART2 (RX2/TX2) - works alongside Loco + Flow
make esp32_uart_bridge_uart2_defconfig
make -j"$(nproc)"
```

The artifact is `build/cf21bl.bin` in both cases. Both configurations are
`configs/cf21bl_defconfig` plus the forced load; they differ only in the port:

```text
CONFIG_DECK_ESP32_UART_BRIDGE=y
CONFIG_DECK_ESP32_UART_BRIDGE_ON_UART1=y   # or ..._ON_UART2=y
CONFIG_DECK_FORCE="esp32UartBridge"
# CONFIG_DEBUG_PRINT_ON_UART1 is not set   # UART1 variant only
```

`CONFIG_DECK_ESP32_UART_BRIDGE_ON_UART1` depends on `!DEBUG_PRINT_ON_UART1`:
that variant owns UART1, so the console cannot be routed to the same port. The
dependency sits on the port choice rather than on the deck symbol, so console
on UART1 plus bridge on UART2 remains a valid combination.

### Other platforms

Both configurations select `CONFIG_PLATFORM_CF21BL=y`. For a plain Crazyflie
2.1 replace that with `CONFIG_PLATFORM_CF2=y` and drop the two brushless motor
lines; the driver itself is platform independent. Note that on the CF2 the
UART2 pins `PA2`/`PA3` are also the `M1`/`M4` motor pins on the deck
connector — check your motor mapping before picking the UART2 variant there.

## Behaviour without an ESP32 attached

Because the deck is forced, the driver is initialised even when nothing is
connected. `esp32UartBridgeInit()` only configures UART1 and starts the receive
task, it never waits for the peer. The task blocks on the UART1 queue, so an
idle link costs no CPU time and cannot trip the watchdog, and the self test
reports success as long as the task was created. The log variables stay at `0`.
`param deck.esp32UartBridge` is `1` once the driver is up.

## Reading the values

```bash
python3 examples/esp32_uart_bridge_log.py --duration 10
```

See `examples/esp32_uart_bridge_log.py`. Declaring the variables with `LOG_ADD`
only puts them in the log TOC; the ground station still has to subscribe to
them with a log configuration, which is what the example does (all four in one
block at 100 ms).

## Unit tests

```bash
rake unit FILES=test/deck/drivers/src/test_esp32UartBridgeParser.c
```
