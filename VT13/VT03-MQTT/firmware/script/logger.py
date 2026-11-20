#!/usr/bin/env python3
"""Read tagged firmware logs from a serial port and optionally save CSV data."""

import serial
import serial.tools.list_ports
import argparse
import sys
import time
import os
from typing import Optional, Set, TextIO

# ANSI Control Codes
class ANSI:
    # Cursor control
    HIDE_CURSOR = '\033[?25l'
    SHOW_CURSOR = '\033[?25h'
    CLEAR_SCREEN = '\033[2J'
    HOME = '\033[H'
    SAVE_POS = '\033[s'
    RESTORE_POS = '\033[u'

    # Colors
    RESET = '\033[0m'
    BOLD = '\033[1m'

    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    MAGENTA = '\033[95m'
    BLUE = '\033[94m'

    @staticmethod
    def move_to(row, col):
        return f'\033[{row};{col}H'

    @staticmethod
    def clear_line():
        return '\033[2K'

# Tag configuration
ALL_TAGS = ['SYS', 'CMD', 'CHA', 'GIM', 'SHO', 'SEN', 'MOT', 'IMU', 'CAN', 'VIS', 'RC', 'DEBUG']

TAG_COLORS = {
    'SYS': ANSI.CYAN,
    'CMD': ANSI.GREEN,
    'CHA': ANSI.YELLOW,
    'GIM': ANSI.MAGENTA,
    'SHO': ANSI.BLUE,
    'SEN': ANSI.RED,
    'MOT': ANSI.CYAN,
    'IMU': ANSI.GREEN,
    'CAN': ANSI.YELLOW,
    'VIS': ANSI.MAGENTA,
    'RC': ANSI.GREEN,
    'DEBUG': '\033[90m',
}

# Known field headers
TAG_HEADERS = {
    'RC': ['frame_cnt', 'ch0', 'ch1', 'ch2', 'ch3', 'ch4', 'sw_R', 'sw_L'],
    'CMD': ['spin', 'align', 'pid', 'yaw_err', 'c_vx', 'c_vy', 'c_wz', 'rc_ch2', 'yaw_tgt', 'yaw_cur', 'wz_cmd'],
    'IMU': ['g_yaw', 'g_pitch', 'g_roll', 'yaw_tot', 'rnd', 'g_gx', 'g_gy', 'g_gz',
            'c_yaw', 'c_pitch', 'c_roll', 'c_gx', 'c_gy', 'c_gz', 'c_ax', 'c_ay', 'c_az', 'c_temp', 'st'],
    'GIM_PITCH': ['ang_tgt', 'ang_cur', 'spd_rpm', 'cmd', 'error', 'rate'],
    'GIM_YAW': ['ang_tgt', 'ang_cur', 'spd_rpm', 'cmd_cur', 'cmd_spd', 'rate', 'error', 'g_gz', 'c_gz'],
    'GIM_ENCODER': ['yaw_raw', 'pitch_raw', 'yaw_tgt', 'pitch_tgt'],
    'CAN': ['ch1', 'rx_total1', 'last_id1', 'rx_rate1', 'ch2', 'rx_total2', 'last_id2', 'rx_rate2'],
    'MOT': ['motor_id', 'type', 'online', 'age_ms'],
}

class SmartLogger:
    """Ultra-fast logger with fixed header display"""

    def __init__(self, port, baudrate, tags, save_file=None, auto_save=False):
        self.port = port
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None

        # Tag filtering
        if tags == 'all':
            self.active_tags: Set[str] = set(ALL_TAGS)
        else:
            self.active_tags: Set[str] = set(tags.split(','))

        # File saving
        if auto_save:
            import os
            from datetime import datetime
            script_dir = os.path.dirname(os.path.abspath(__file__))
            project_root = os.path.dirname(script_dir)
            logs_dir = os.path.join(project_root, 'logs')
            os.makedirs(logs_dir, exist_ok=True)
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.save_file = os.path.join(logs_dir, f'smartlog_{timestamp}.csv')
            print(f"{ANSI.CYAN}[Auto-save enabled: {self.save_file}]{ANSI.RESET}")
        else:
            self.save_file = save_file

        self.csv_file: Optional[TextIO] = None

        # Display state
        self.start_time = None
        self.line_count = 0
        self.header_drawn = False
        self.current_tag = None

        # Get terminal size
        self.update_terminal_size()

    def update_terminal_size(self):
        """Get current terminal dimensions"""
        try:
            size = os.get_terminal_size()
            self.term_width = size.columns
            self.term_height = size.lines
        except:
            self.term_width = 120
            self.term_height = 40

    def connect(self):
        """Connect to serial port"""
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=0.001)
            time.sleep(0.3)
            return True
        except Exception as e:
            print(f"{ANSI.RED}[ERROR] Failed to open {self.port}: {e}{ANSI.RESET}")
            return False

    def reconnect(self, infinite_retry=False):
        """Attempt to reconnect to serial port

        Args:
            infinite_retry: If True, keep trying until successful or Ctrl+C
        """
        if self.serial:
            try:
                self.serial.close()
            except:
                pass
            self.serial = None

        # Print reconnection message
        sys.stdout.write('\033[r')  # Reset scroll region
        print(f"\n{ANSI.YELLOW}[Serial port disconnected - Reconnecting...]{ANSI.RESET}", flush=True)
        print(f"{ANSI.YELLOW}[Press Ctrl+C to exit]{ANSI.RESET}", flush=True)

        # Try to reconnect
        attempt = 0
        max_attempts = None if infinite_retry else 5

        try:
            while True:
                attempt += 1
                if max_attempts:
                    print(f"{ANSI.YELLOW}[Attempt {attempt}/{max_attempts}]{ANSI.RESET}", flush=True)
                else:
                    print(f"{ANSI.YELLOW}[Attempt {attempt}...]{ANSI.RESET}", flush=True)

                if self.connect():
                    print(f"{ANSI.GREEN}[Connected to {self.port}]{ANSI.RESET}", flush=True)
                    time.sleep(1)  # Brief pause before resuming

                    # Reinitialize terminal display
                    sys.stdout.write(ANSI.CLEAR_SCREEN)
                    sys.stdout.write(ANSI.HOME)
                    sys.stdout.write(ANSI.HIDE_CURSOR)

                    # Redraw header if we had one
                    if self.current_tag:
                        self.draw_header(self.current_tag)
                        sys.stdout.write(f'\033[6;{self.term_height}r')
                        sys.stdout.write(ANSI.move_to(6, 1))

                    sys.stdout.flush()
                    return True

                # Check if we should stop trying
                if max_attempts and attempt >= max_attempts:
                    break

                time.sleep(1)  # Wait before next attempt

        except KeyboardInterrupt:
            print(f"\n{ANSI.YELLOW}[Reconnection cancelled by user]{ANSI.RESET}")
            raise

        print(f"{ANSI.RED}[Failed to reconnect after {attempt} attempts]{ANSI.RESET}")
        return False

    def parse_csv(self, line):
        """Parse CSV line: TAG,timestamp,field1,field2,..."""
        parts = line.strip().split(',')
        if len(parts) < 2:
            return None

        tag = parts[0]

        # Handle GIM sub-formats (PITCH, YAW, ENCODER)
        if tag == 'GIM' and len(parts) > 2:
            # Check if third field is a sub-format identifier
            sub_format = parts[2]
            if sub_format in ['PITCH', 'YAW', 'ENCODER']:
                tag = f"GIM_{sub_format}"
                parts = [tag, parts[1]] + parts[3:]  # Reformat to remove sub-format field

        if tag not in ALL_TAGS and not tag.startswith('GIM_'):
            return None

        try:
            timestamp = int(parts[1])
            values = []
            for field in parts[2:]:
                try:
                    if '.' in field:
                        values.append(float(field))
                    else:
                        values.append(int(field))
                except:
                    values.append(field)
            return tag, timestamp, values
        except:
            return None

    def draw_header(self, tag):
        """Draw fixed header at top of screen"""
        tag_color = TAG_COLORS.get(tag, ANSI.GREEN)

        # Get headers for this tag
        headers = TAG_HEADERS.get(tag, None)
        if not headers:
            return

        # Go to line 1 and draw border
        sys.stdout.write(ANSI.move_to(1, 1))
        sys.stdout.write(ANSI.clear_line())
        sys.stdout.write(f"{ANSI.BOLD}{ANSI.CYAN}{'='*min(self.term_width, 120)}{ANSI.RESET}")

        # Go to line 2 and draw headers
        sys.stdout.write(ANSI.move_to(2, 1))
        sys.stdout.write(ANSI.clear_line())
        header_line = f"{tag_color}[{tag:>11}]{ANSI.RESET} │ "
        header_line += "  ".join([f"{h:>10}" for h in headers])
        sys.stdout.write(header_line)

        # Go to line 3 and draw separator
        sys.stdout.write(ANSI.move_to(3, 1))
        sys.stdout.write(ANSI.clear_line())
        sys.stdout.write(f"{ANSI.CYAN}{'─'*min(self.term_width, 120)}{ANSI.RESET}")

        sys.stdout.flush()

        self.header_drawn = True
        self.current_tag = tag

    def format_values(self, values):
        """Format values for display"""
        result = []
        for v in values:
            if isinstance(v, float):
                result.append(f"{v:>10.2f}")
            elif isinstance(v, int):
                result.append(f"{v:>10d}")
            else:
                result.append(f"{str(v):>10}")
        return "  ".join(result)

    def display_data(self, tag, timestamp, values):
        """Display one line of data"""
        # Initialize start time and draw header
        if self.start_time is None:
            self.start_time = timestamp

            # Draw header BEFORE setting scroll region
            self.draw_header(tag)

            # NOW set scrolling region (line 6 to bottom, leaving lines 4-5 empty)
            sys.stdout.write(f'\033[6;{self.term_height}r')

            # Move to line 6 (start of data area, lines 4-5 are empty for spacing)
            sys.stdout.write(ANSI.move_to(6, 1))
            sys.stdout.flush()

        # Update header if tag changed (redraw header outside scroll region)
        if tag != self.current_tag:
            # Temporarily disable scroll region
            sys.stdout.write('\033[r')
            self.draw_header(tag)
            # Re-enable scroll region (line 6 to bottom)
            sys.stdout.write(f'\033[6;{self.term_height}r')
            sys.stdout.flush()

        # Format line
        tag_color = TAG_COLORS.get(tag, ANSI.GREEN)
        line = f"{tag_color}[{tag:>11}]{ANSI.RESET} │ {self.format_values(values)}"

        # Print data (will scroll in region)
        print(line, flush=True)

        self.line_count += 1

    def run(self):
        """Main loop"""
        if not self.connect():
            # Initial connection failed, try reconnect
            if not self.reconnect():
                print(f"{ANSI.RED}[FATAL] Unable to connect to serial port{ANSI.RESET}")
                return

        assert self.serial is not None, "Serial port not connected"

        # Open save file if needed
        if self.save_file:
            self.csv_file = open(self.save_file, 'w', buffering=1)  # Line buffered

        # Setup terminal
        sys.stdout.write(ANSI.CLEAR_SCREEN)
        sys.stdout.write(ANSI.HOME)
        sys.stdout.write(ANSI.HIDE_CURSOR)
        sys.stdout.flush()

        # Scroll region will be set when first data arrives (in display_data)

        try:
            while True:
                try:
                    # Read line
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if not line:
                        continue

                    # Parse
                    result = self.parse_csv(line)
                    if not result:
                        continue

                    tag, timestamp, values = result

                    # Filter by active tags
                    base_tag = tag.split('_')[0]  # For GIM_PITCH -> GIM
                    if base_tag not in self.active_tags and tag not in self.active_tags:
                        continue

                    # Display
                    self.display_data(tag, timestamp, values)

                    # Save to file
                    if self.csv_file:
                        self.csv_file.write(line + '\n')

                except KeyboardInterrupt:
                    raise
                except (serial.SerialException, OSError) as e:
                    # Serial port disconnected, try to reconnect indefinitely
                    try:
                        if not self.reconnect(infinite_retry=True):
                            # Should not reach here with infinite_retry=True
                            # unless connect() keeps failing
                            time.sleep(2)
                            continue
                    except KeyboardInterrupt:
                        # User cancelled during reconnection
                        raise
                    # Successfully reconnected, continue loop
                    assert self.serial is not None  # For type checker
                    continue
                except Exception:
                    continue  # Ignore other errors, keep going

        except KeyboardInterrupt:
            print(f"\n{ANSI.YELLOW}Stopped by user{ANSI.RESET}")

        finally:
            # Cleanup - reset scrolling region
            sys.stdout.write('\033[r')  # Reset scroll region to full screen
            sys.stdout.write(ANSI.SHOW_CURSOR)
            sys.stdout.write('\n')  # Move to next line
            sys.stdout.flush()

            if self.csv_file:
                self.csv_file.close()

            if self.serial:
                self.serial.close()

            print(f"{ANSI.GREEN}Total lines: {self.line_count}{ANSI.RESET}")

def auto_detect_port():
    """Auto-detect serial port"""
    ports = serial.tools.list_ports.comports()
    if not ports:
        return None

    # Prefer USB serial
    for p in ports:
        if 'usb' in p.device.lower() or 'ACM' in p.device:
            return p.device

    return ports[0].device

def interactive_tag_selection():
    """Interactive menu for tag selection"""
    print(f"\n{ANSI.BOLD}{ANSI.CYAN}{'='*80}{ANSI.RESET}")
    print(f"{ANSI.BOLD}Smart Logger - Tag Selection{ANSI.RESET}")
    print(f"{ANSI.CYAN}{'='*80}{ANSI.RESET}\n")

    TAG_DESCRIPTIONS = {
        'SYS': 'System initialization and status',
        'CMD': 'Command controller and spin mode',
        'CHA': 'Chassis motor control',
        'GIM': 'Gimbal pitch/yaw control',
        'SHO': 'Shooter mechanism',
        'SEN': 'Sentry mode controller',
        'MOT': 'Motor driver diagnostics',
        'IMU': 'IMU sensor data (dual IMU)',
        'CAN': 'CAN bus diagnostics',
        'VIS': 'Vision system communication',
        'RC': 'Remote control input',
        'DEBUG': 'Debug messages'
    }

    print(f"{ANSI.BOLD}Available Tags:{ANSI.RESET}")
    for i, tag in enumerate(ALL_TAGS, 1):
        color = TAG_COLORS.get(tag, '')
        desc = TAG_DESCRIPTIONS.get(tag, '')
        print(f"  {i:>2}. {color}{tag:>5}{ANSI.RESET} - {desc}")

    print(f"\n{ANSI.BOLD}Selection Options:{ANSI.RESET}")
    print(f"  - Enter numbers: {ANSI.GREEN}1,3,5{ANSI.RESET} (select multiple)")
    print(f"  - Enter 'all': {ANSI.GREEN}all{ANSI.RESET} (select all tags)")
    print(f"  - Press Enter: Use default {ANSI.GREEN}RC{ANSI.RESET}")

    while True:
        try:
            selection = input(f"\n{ANSI.BOLD}Select tags >{ANSI.RESET} ").strip()

            if not selection:
                return ['RC']  # Default to RC only

            if selection.lower() == 'all':
                return ALL_TAGS

            # Parse numbers
            indices = [int(x.strip()) for x in selection.split(',')]
            selected_tags = []
            for idx in indices:
                if 1 <= idx <= len(ALL_TAGS):
                    selected_tags.append(ALL_TAGS[idx - 1])
                else:
                    print(f"{ANSI.RED}Invalid index: {idx}{ANSI.RESET}")
                    break
            else:
                if selected_tags:
                    print(f"\n{ANSI.GREEN}Selected:{ANSI.RESET}", end=" ")
                    for tag in selected_tags:
                        color = TAG_COLORS.get(tag, '')
                        print(f"{color}{tag}{ANSI.RESET}", end=" ")
                    print()
                    return selected_tags

        except ValueError:
            print(f"{ANSI.RED}Invalid input. Please enter numbers separated by commas.{ANSI.RESET}")
        except KeyboardInterrupt:
            print(f"\n{ANSI.YELLOW}Cancelled{ANSI.RESET}")
            sys.exit(0)

def list_available_tags():
    """Print list of all available tags"""
    print(f"\n{ANSI.BOLD}{ANSI.CYAN}{'='*80}{ANSI.RESET}")
    print(f"{ANSI.BOLD}Available Tags:{ANSI.RESET}\n")

    TAG_DESCRIPTIONS = {
        'SYS': 'System initialization and status',
        'CMD': 'Command controller and spin mode',
        'CHA': 'Chassis motor control',
        'GIM': 'Gimbal pitch/yaw control',
        'SHO': 'Shooter mechanism',
        'SEN': 'Sentry mode controller',
        'MOT': 'Motor driver diagnostics',
        'IMU': 'IMU sensor data (dual IMU)',
        'CAN': 'CAN bus diagnostics',
        'VIS': 'Vision system communication',
        'RC': 'Remote control input',
        'DEBUG': 'Debug messages'
    }

    for tag in ALL_TAGS:
        color = TAG_COLORS.get(tag, '')
        desc = TAG_DESCRIPTIONS.get(tag, '')
        print(f"  {color}{tag:>5}{ANSI.RESET} - {desc}")

    print(f"\n{ANSI.BOLD}Usage:{ANSI.RESET}")
    print(f"  python3 script/logger.py --tags {ANSI.GREEN}CMD,GIM{ANSI.RESET}")
    print(f"  python3 script/logger.py --tags {ANSI.GREEN}all{ANSI.RESET}")
    print()

def main():
    parser = argparse.ArgumentParser(
        description='Smart Logger - High-performance terminal logger',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=f"""
{ANSI.BOLD}Examples:{ANSI.RESET}
  %(prog)s                          # Interactive tag selection
  %(prog)s --tags RC                # Monitor RC only
  %(prog)s --tags CMD,GIM,IMU       # Monitor multiple tags
  %(prog)s --save data.csv          # Save data to file
  %(prog)s --auto-save              # Auto-save to logs/ directory
  %(prog)s --list-tags              # List all available tags
        """
    )

    parser.add_argument('port', nargs='?', help='Serial port (auto-detect if not specified)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--tags', default=None, help='Comma-separated tags or "all" (default: interactive)')
    parser.add_argument('--save', metavar='FILE', help='Save data to CSV file')
    parser.add_argument('--auto-save', action='store_true', help='Auto-save to logs/ directory with timestamp')
    parser.add_argument('--list-tags', action='store_true', help='List all available tags and exit')

    args = parser.parse_args()

    # Handle special commands
    if args.list_tags:
        list_available_tags()
        return

    # Tag selection
    if args.tags:
        tags = args.tags
    else:
        selected = interactive_tag_selection()
        tags = ','.join(selected)

    # Auto-detect port
    port = args.port
    if not port:
        port = auto_detect_port()
        if not port:
            print(f"{ANSI.RED}[ERROR] No serial ports found{ANSI.RESET}")
            return
        print(f"{ANSI.GREEN}[Auto-detect] Using port: {port}{ANSI.RESET}")

    # Create and run logger
    logger = SmartLogger(port, args.baud, tags, args.save, args.auto_save)
    logger.run()

if __name__ == '__main__':
    main()
