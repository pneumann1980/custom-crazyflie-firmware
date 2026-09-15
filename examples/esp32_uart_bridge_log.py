#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2026 Bitcraze AB
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program. If not, see <https://www.gnu.org/licenses/>.
"""
Ground station side of the ESP32 UART bridge deck.

Subscribes to the four log variables published by
src/deck/drivers/src/ESP32UartBridgeDeck.c and prints them as they arrive:

    esp32Uart.var1 .. esp32Uart.var4

All four variables share a single log configuration, so every printed row is
one consistent sample taken by the firmware at the same instant. Requires a
firmware built with:

    make esp32_uart_bridge_defconfig
    make -j"$(nproc)"

Usage:

    python3 examples/esp32_uart_bridge_log.py
    python3 examples/esp32_uart_bridge_log.py --uri radio://0/80/2M/E7E7E7E7E7
    python3 examples/esp32_uart_bridge_log.py --period 100 --duration 10

The URI defaults to the CFLIB_URI environment variable and falls back to
radio://0/80/2M/E7E7E7E7E7, the same convention the cflib examples use.
"""
import argparse
import logging
import sys
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

DEFAULT_URI = 'radio://0/80/2M/E7E7E7E7E7'
LOG_GROUP = 'esp32Uart'
LOG_VARIABLES = ['var1', 'var2', 'var3', 'var4']

# Diagnostic counters of the firmware side parser. Optional: they are only
# subscribed with --stats because they live in a second log block.
STATS_VARIABLES = ['nOk', 'nErr', 'nUnknown']

logging.basicConfig(level=logging.ERROR)


class Esp32UartBridgeLogger:
    """Print the esp32Uart log group of a connected Crazyflie."""

    def __init__(self, period_ms, with_stats):
        self._period_ms = period_ms
        self._with_stats = with_stats
        self._configs = []
        self._samples = 0
        self._errors = 0

    def run(self, uri, duration_s):
        """Connect, subscribe and print until duration_s has elapsed.

        Returns a process exit code.
        """
        print(f'Connecting to {uri} ...')
        try:
            with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
                print(f'Connected to {uri}')
                if not self._subscribe(scf):
                    return 1
                self._print_header()
                self._wait(duration_s)
                self._unsubscribe()
        except Exception as error:
            # cflib raises a plain Exception when the link cannot be opened and
            # embeds a full traceback in the message; the first line is the
            # part a user can act on.
            reason = str(error).splitlines()[0] if str(error) else type(error).__name__
            print(f'Could not connect to {uri}: {reason}', file=sys.stderr)
            print('Check that the Crazyflie is powered on, in range and that '
                  'the radio dongle is plugged in.', file=sys.stderr)
            return 1

        print(f'\nReceived {self._samples} sample(s), {self._errors} log error(s).')
        if self._samples == 0:
            print('No samples received. Is the log configuration still running?',
                  file=sys.stderr)
            return 1
        return 0

    def _subscribe(self, scf):
        """Create and start the log blocks. Returns False on failure."""
        # (log block name, variables, fetch type matching the firmware LOG_ADD)
        blocks = [(LOG_GROUP, LOG_VARIABLES, 'uint16_t')]
        if self._with_stats:
            blocks.append((LOG_GROUP + '_stats', STATS_VARIABLES, 'uint32_t'))

        for name, variables, fetch_as in blocks:
            config = LogConfig(name=name, period_in_ms=self._period_ms)
            for variable in variables:
                config.add_variable(f'{LOG_GROUP}.{variable}', fetch_as)

            config.data_received_cb.add_callback(self._on_data)
            config.error_cb.add_callback(self._on_error)

            try:
                scf.cf.log.add_config(config)
            except KeyError as error:
                print(f'Log variable missing on the Crazyflie: {error}', file=sys.stderr)
                print('The firmware does not expose the esp32Uart log group. Build and '
                      'flash it with "make esp32_uart_bridge_defconfig && make".',
                      file=sys.stderr)
                return False
            except AttributeError as error:
                print(f'Invalid log configuration: {error}', file=sys.stderr)
                return False

            config.start()
            self._configs.append(config)

        return True

    def _unsubscribe(self):
        for config in self._configs:
            try:
                config.stop()
                config.delete()
            except Exception as error:  # noqa: BLE001 - best effort cleanup
                print(f'Failed to remove log block {config.name}: {error}',
                      file=sys.stderr)
        self._configs = []

    def _wait(self, duration_s):
        if duration_s <= 0:
            print('Logging until interrupted with Ctrl-C ...')
        deadline = time.time() + duration_s
        try:
            while duration_s <= 0 or time.time() < deadline:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print('\nInterrupted.')

    def _print_header(self):
        columns = ['host time', 'cf time [ms]'] + [f'{LOG_GROUP}.{v}' for v in LOG_VARIABLES]
        if self._with_stats:
            columns += [f'{LOG_GROUP}.{v}' for v in STATS_VARIABLES]
        print()
        print('  '.join(f'{c:>14}' for c in columns))
        print('-' * (16 * len(columns)))

    def _on_data(self, timestamp, data, logconf):
        """Called from the cflib callback thread for every received sample."""
        if logconf.name != LOG_GROUP:
            # Statistics block, printed on its own line to keep rows aligned.
            values = '  '.join(f'{data[f"{LOG_GROUP}.{v}"]:>14d}' for v in STATS_VARIABLES)
            print(f'{"":>14}  {timestamp:>14d}  {values}   (parser counters)')
            return

        self._samples += 1
        host_time = time.strftime('%H:%M:%S') + f'.{int(time.time() * 1000) % 1000:03d}'
        values = '  '.join(
            f'0x{data[f"{LOG_GROUP}.{v}"]:04X} ({data[f"{LOG_GROUP}.{v}"]:5d})'
            for v in LOG_VARIABLES)
        print(f'{host_time:>14}  {timestamp:>14d}  {values}')

    def _on_error(self, logconf, msg):
        self._errors += 1
        print(f'Log error in block "{logconf.name}": {msg}', file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(
        description='Subscribe to the esp32Uart log group of the ESP32 UART bridge deck.')
    parser.add_argument('--uri', default=uri_helper.uri_from_env(default=DEFAULT_URI),
                        help='Crazyflie URI (default: $CFLIB_URI or %s)' % DEFAULT_URI)
    parser.add_argument('--period', type=int, default=100, metavar='MS',
                        help='Log period in ms, 10..2550 in steps of 10 (default: 100)')
    parser.add_argument('--duration', type=float, default=10.0, metavar='S',
                        help='Seconds to log, 0 logs until Ctrl-C (default: 10)')
    parser.add_argument('--stats', action='store_true',
                        help='Also subscribe to the parser diagnostic counters')
    args = parser.parse_args()

    if not 10 <= args.period <= 2550:
        parser.error('--period must be between 10 and 2550 ms')

    cflib.crtp.init_drivers()

    logger = Esp32UartBridgeLogger(period_ms=args.period, with_stats=args.stats)
    return logger.run(args.uri, args.duration)


if __name__ == '__main__':
    sys.exit(main())
