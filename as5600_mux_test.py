#!/usr/bin/env python3
"""Read an AS5600 through a selected I2C multiplexer port."""

import argparse
import sys
import time

from smbus2 import SMBus, i2c_msg


BUS_NUMBER = 1
AS5600_ADDRESS = 0x36
MUX_ADDRESSES = range(0x70, 0x78)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Select one I2C mux port and sample an AS5600 encoder."
    )
    parser.add_argument(
        "--port",
        type=int,
        choices=range(8),
        default=0,
        metavar="0-7",
        help="multiplexer port to select (default: 0)",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=10,
        help="number of angle samples to read (default: 10)",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.1,
        help="seconds between samples (default: 0.1)",
    )
    args = parser.parse_args()
    if args.samples < 1:
        parser.error("--samples must be at least 1")
    if args.interval < 0:
        parser.error("--interval cannot be negative")
    return args


def raw_read(bus: SMBus, address: int) -> int:
    message = i2c_msg.read(address, 1)
    bus.i2c_rdwr(message)
    return list(message)[0]


def find_mux(bus: SMBus) -> tuple[int, int]:
    responding = []
    for address in MUX_ADDRESSES:
        try:
            control = raw_read(bus, address)
        except OSError:
            continue
        responding.append((address, control))

    if not responding:
        raise RuntimeError("No I2C multiplexer responded at addresses 0x70-0x77")
    if len(responding) > 1:
        addresses = ", ".join(f"0x{address:02X}" for address, _ in responding)
        raise RuntimeError(f"Multiple possible multiplexers responded: {addresses}")
    return responding[0]


def read_as5600(bus: SMBus) -> tuple[int, int, int]:
    write = i2c_msg.write(AS5600_ADDRESS, [0x0B])
    read = i2c_msg.read(AS5600_ADDRESS, 5)
    bus.i2c_rdwr(write, read)
    data = list(read)
    status = data[0]
    raw_angle = ((data[1] & 0x0F) << 8) | data[2]
    angle = ((data[3] & 0x0F) << 8) | data[4]
    return status, raw_angle, angle


def main() -> int:
    args = parse_args()

    try:
        with SMBus(BUS_NUMBER) as bus:
            mux_address, original_control = find_mux(bus)
            print(
                f"Mux detected at 0x{mux_address:02X}; "
                f"original control=0x{original_control:02X}"
            )

            try:
                port_mask = 1 << args.port
                bus.write_byte(mux_address, port_mask)
                time.sleep(0.01)
                selected_control = raw_read(bus, mux_address)
                print(
                    f"Selected port {args.port}; "
                    f"mux control=0x{selected_control:02X}"
                )
                if selected_control != port_mask:
                    raise RuntimeError(
                        f"Mux did not retain the port-{args.port} selection"
                    )

                for sample_number in range(1, args.samples + 1):
                    status, raw_angle, angle = read_as5600(bus)
                    print(
                        f"sample {sample_number:02d}: status=0x{status:02X} "
                        f"MD={(status >> 5) & 1} ML={(status >> 4) & 1} "
                        f"MH={(status >> 3) & 1} raw={raw_angle:4d} "
                        f"angle={angle:4d} ({angle * 360 / 4096:7.2f} deg)"
                    )
                    time.sleep(args.interval)
            finally:
                bus.write_byte(mux_address, original_control)
                print(f"Restored mux control to 0x{original_control:02X}")
    except (OSError, RuntimeError) as error:
        print(f"I2C test failed: {error}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
