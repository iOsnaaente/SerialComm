"""
Simple UART reader for debugging SerialComm traffic.

Dependencies:
	pip install pyserial
"""

import argparse
import sys
import time

import serial


def main() -> int:
	parser = argparse.ArgumentParser(description="Read and print UART data.")
	parser.add_argument("port", help="Serial port, e.g. /dev/ttyUSB0")
	parser.add_argument(
		"--baudrate",
		type=int,
		default=921600,
		help="Baud rate (default: 921600)",
	)
	parser.add_argument(
		"--timeout",
		type=float,
		default=0.1,
		help="Read timeout in seconds (default: 0.1)",
	)
	args = parser.parse_args()

	try:
		ser = serial.Serial(
			port=args.port,
			baudrate=args.baudrate,
			timeout=args.timeout,
		)
	except Exception as exc:
		print(f"[UART] failed to open {args.port}: {exc}", file=sys.stderr)
		return 1

	print(f"[UART] listening on {args.port} @ {args.baudrate}")

	try:
		while True:
			data = ser.read(256)
			if data:
				print(data.decode("utf-8", errors="replace"), end="", flush=True)
			else:
				time.sleep(0.01)
	except KeyboardInterrupt:
		print("\n[UART] stopped")
	finally:
		ser.close()

	return 0


if __name__ == "__main__":
	raise SystemExit(main())
