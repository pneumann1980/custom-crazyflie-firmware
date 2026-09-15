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
| Port | UART1, 115200 baud, 8N1, no flow control |
| Log group | `esp32Uart` |

## Wiring

The Crazyflie deck port is 3.3 V only and is **not** 5 V tolerant. Use an ESP32
board whose UART pins run at 3.3 V, or insert a level shifter.

```text
ESP32 TX  -> Crazyflie UART1 RX / RX1 (PC11)
ESP32 RX  -> Crazyflie UART1 TX / TX1 (PC10), only if ever needed
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

The counters are not part of the requested interface, but `nErr` rising while
`nOk` stalls is the quickest way to tell a wiring or baudrate problem from a
silent ESP32.

## Build

```bash
make esp32_uart_bridge_defconfig
make -j"$(nproc)"
```

The artifact is `build/cf21bl.bin`. The configuration is
`configs/esp32_uart_bridge_defconfig`, which is `configs/cf21bl_defconfig` plus:

```text
CONFIG_DECK_ESP32_UART_BRIDGE=y
CONFIG_DECK_FORCE="esp32UartBridge"
# CONFIG_DEBUG_PRINT_ON_UART1 is not set
```

`CONFIG_DECK_ESP32_UART_BRIDGE` depends on `!DEBUG_PRINT_ON_UART1`: the deck
owns UART1, so the console cannot be routed to the same port.

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
