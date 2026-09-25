#!/usr/bin/env python3
"""
Serial Data Collector for Caster Robot Arm MCU
===============================================
Records joint trajectory data in VOFA FireWater format over serial port.

Data format: comma-separated values ending with newline.
Header: time,q0,q1,q2,q3,q4,q5,qd0,qd1,qd2,qd3,qd4,qd5,tau0,tau1,tau2,tau3,tau4,tau5

Usage:
    python tools/serial_record.py -p /dev/ttyUSB0 -o data/output.csv -d 35

Requirements:
    pip install pyserial
"""

import argparse
import csv
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

try:
    import serial
except ImportError:
    print("=" * 60)
    print("ERROR: pyserial is not installed.")
    print("Install it with:  pip install pyserial")
    print("=" * 60)
    sys.exit(1)

# Expected CSV header for VOFA FireWater format
EXPECTED_HEADER = [
    "time", "q0", "q1", "q2", "q3", "q4", "q5",
    "qd0", "qd1", "qd2", "qd3", "qd4", "qd5",
    "tau0", "tau1", "tau2", "tau3", "tau4", "tau5"
]
NUM_COLUMNS = len(EXPECTED_HEADER)  # 19


class GracefulExit:
    """Handle Ctrl+C gracefully by setting a flag."""
    
    def __init__(self):
        self.should_exit = False
        signal.signal(signal.SIGINT, self._handler)
        signal.signal(signal.SIGTERM, self._handler)
    
    def _handler(self, signum, frame):
        print("\n\n⚠️  Ctrl+C received. Finishing recording...")
        self.should_exit = True


def validate_line(line: str) -> list[float] | None:
    """
    Validate and parse a VOFA FireWater data line.
    
    Returns parsed float values if valid, None otherwise.
    """
    line = line.strip()
    
    # Skip empty lines
    if not line:
        return None
    
    parts = line.split(",")
    
    # Must have exactly 19 comma-separated values
    if len(parts) != NUM_COLUMNS:
        return None
    
    # Try parsing all values as floats
    try:
        values = [float(p) for p in parts]
    except ValueError:
        return None
    
    return values


def print_progress(sample_count: int, elapsed: float, duration: float):
    """Print progress information."""
    remaining = max(0, duration - elapsed)
    progress_pct = min(100, (elapsed / duration) * 100)
    bar_len = 30
    filled = int(bar_len * progress_pct / 100)
    bar = "█" * filled + "░" * (bar_len - filled)
    
    sys.stdout.write(
        f"\r📊 [{bar}] {progress_pct:5.1f}% | "
        f"Samples: {sample_count:5d} | "
        f"Elapsed: {elapsed:5.1f}s / {duration}s | "
        f"Rate: {sample_count / max(elapsed, 0.01):5.1f} Hz"
    )
    sys.stdout.flush()


def print_summary(sample_count: int, elapsed: float, skipped: int, output_path: str):
    """Print recording summary."""
    avg_rate = sample_count / elapsed if elapsed > 0 else 0
    
    print("\n")
    print("=" * 60)
    print("📈 RECORDING SUMMARY")
    print("=" * 60)
    print(f"  Total samples collected : {sample_count}")
    print(f"  Skipped/invalid lines   : {skipped}")
    print(f"  Actual duration         : {elapsed:.2f} seconds")
    print(f"  Average sample rate     : {avg_rate:.2f} Hz")
    print(f"  Output file             : {output_path}")
    print(f"  File size               : {Path(output_path).stat().st_size / 1024:.1f} KB")
    print("=" * 60)


def record_serial(port: str, baud: int, output_path: str, duration: float):
    """Main recording function."""
    
    # Ensure output directory exists
    out_dir = Path(output_path).parent
    out_dir.mkdir(parents=True, exist_ok=True)
    
    print("=" * 60)
    print("🤖 Caster Robot Arm - Serial Data Recorder")
    print("=" * 60)
    print(f"  Port     : {port}")
    print(f"  Baud     : {baud}")
    print(f"  Output   : {output_path}")
    print(f"  Duration : {duration} seconds")
    print(f"  Columns  : {NUM_COLUMNS} ({', '.join(EXPECTED_HEADER[:6])}, ...)")
    print("=" * 60)
    print()
    
    # Setup graceful exit handler
    exit_handler = GracefulExit()
    
    # Open serial port
    try:
        ser = serial.Serial(port, baud, timeout=1.0)
        print(f"✅ Connected to {port} at {baud} baud")
    except serial.SerialException as e:
        print(f"❌ Failed to open serial port {port}: {e}")
        print("   Check that the port exists and permissions are correct.")
        print("   You may need: sudo chmod 666 {port}")
        sys.exit(1)
    
    # Open output CSV file
    try:
        csvfile = open(output_path, "w", newline="")
        writer = csv.writer(csvfile)
        writer.writerow(EXPECTED_HEADER)
    except IOError as e:
        print(f"❌ Failed to open output file {output_path}: {e}")
        ser.close()
        sys.exit(1)
    
    # Recording loop
    sample_count = 0
    skipped_count = 0
    start_time = time.monotonic()
    last_progress_time = start_time
    header_seen = False
    
    print("\n🔴 Recording started. Press Ctrl+C to stop early.\n")
    
    try:
        while not exit_handler.should_exit:
            # Check if duration exceeded
            elapsed = time.monotonic() - start_time
            if elapsed >= duration:
                print(f"\n\n⏱️  Recording duration ({duration}s) reached.")
                break
            
            # Read a line from serial
            try:
                raw_line = ser.readline()
            except serial.SerialException as e:
                print(f"\n\n❌ Serial read error: {e}")
                break
            
            # Decode bytes to string
            try:
                line = raw_line.decode("utf-8", errors="replace")
            except Exception:
                skipped_count += 1
                continue
            
            # Skip the header line if sent by MCU
            if not header_seen and "time" in line and "q0" in line:
                header_seen = True
                continue
            
            # Try to validate and parse the line
            values = validate_line(line)
            
            if values is None:
                # Not a valid data line (log message, garbage, etc.)
                if line.strip():
                    skipped_count += 1
                continue
            
            # Write valid data row
            writer.writerow(values)
            sample_count += 1
            
            # Print progress every second
            now = time.monotonic()
            if now - last_progress_time >= 1.0:
                print_progress(sample_count, elapsed, duration)
                last_progress_time = now
    
    except Exception as e:
        print(f"\n\n❌ Unexpected error: {e}")
    
    finally:
        # Cleanup
        final_elapsed = time.monotonic() - start_time
        csvfile.flush()
        csvfile.close()
        ser.close()
        
        print_progress(sample_count, final_elapsed, duration)
        print_summary(sample_count, final_elapsed, skipped_count, output_path)
        print("\n✅ Done. File saved successfully.")


def main():
    parser = argparse.ArgumentParser(
        description="Record joint trajectory data from Caster robot arm MCU via serial port.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                                     # Use defaults: /dev/ttyUSB0, 921600, 35s
  %(prog)s -p /dev/ttyACM0 -o my_data.csv      # Custom port and output
  %(prog)s -d 60 -b 115200                     # 60s at 115200 baud
        """
    )
    parser.add_argument(
        "-p", "--port",
        default="/dev/ttyUSB0",
        help="Serial port (default: /dev/ttyUSB0)"
    )
    parser.add_argument(
        "-b", "--baud",
        type=int,
        default=921600,
        help="Baud rate (default: 921600)"
    )
    parser.add_argument(
        "-o", "--output",
        default="/code/Dynamic_Parameter_Identification/Robot-Parameter-Indentification-Simulation/data/output.csv",
        help="Output CSV file path"
    )
    parser.add_argument(
        "-d", "--duration",
        type=float,
        default=35.0,
        help="Recording duration in seconds (default: 35)"
    )
    
    args = parser.parse_args()
    
    # Validate duration
    if args.duration <= 0:
        print("❌ Duration must be positive.")
        sys.exit(1)
    
    # Validate baud rate
    if args.baud <= 0:
        print("❌ Baud rate must be positive.")
        sys.exit(1)
    
    record_serial(args.port, args.baud, args.output, args.duration)


if __name__ == "__main__":
    main()
