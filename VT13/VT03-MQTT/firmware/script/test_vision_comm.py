#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Vision Communication Test Script for Upper Computer
Test USB communication with robomaster-control lower computer
"""

import serial
import struct
import time
import sys

class VisionCommTest:
    def __init__(self, port, baudrate=115200):
        """
        Initialize test program
        :param port: COM port (Windows: 'COM3', Linux: '/dev/ttyACM0')
        :param baudrate: Baud rate (USB VCP typically 115200 or any value)
        """
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            print(f"✓ Successfully connected to {port}")
        except Exception as e:
            print(f"✗ Failed to connect to {port}: {e}")
            sys.exit(1)
        
        self.crc8_table = self._init_crc8_table()
        self.packet_count = 0
        
    def _init_crc8_table(self):
        """Initialize CRC8 lookup table (SHT75 algorithm)"""
        table = [
            0, 49, 98, 83, 196, 245, 166, 151, 185, 136, 219, 234, 125, 76, 31, 46,
            67, 114, 33, 16, 135, 182, 229, 212, 250, 203, 152, 169, 62, 15, 92, 109,
            134, 183, 228, 213, 66, 115, 32, 17, 63, 14, 93, 108, 251, 202, 153, 168,
            197, 244, 167, 150, 1, 48, 99, 82, 124, 77, 30, 47, 184, 137, 218, 235,
            61, 12, 95, 110, 249, 200, 155, 170, 132, 181, 230, 215, 64, 113, 34, 19,
            126, 79, 28, 45, 186, 139, 216, 233, 199, 246, 165, 148, 3, 50, 97, 80,
            187, 138, 217, 232, 127, 78, 29, 44, 2, 51, 96, 81, 198, 247, 164, 149,
            248, 201, 154, 171, 60, 13, 94, 111, 65, 112, 35, 18, 133, 180, 231, 214,
            122, 75, 24, 41, 190, 143, 220, 237, 195, 242, 161, 144, 7, 54, 101, 84,
            57, 8, 91, 106, 253, 204, 159, 174, 128, 177, 226, 211, 68, 117, 38, 23,
            252, 205, 158, 175, 56, 9, 90, 107, 69, 116, 39, 22, 129, 176, 227, 210,
            191, 142, 221, 236, 123, 74, 25, 40, 6, 55, 100, 85, 194, 243, 160, 145,
            71, 118, 37, 20, 131, 178, 225, 208, 254, 207, 156, 173, 58, 11, 88, 105,
            4, 53, 102, 87, 192, 241, 162, 147, 189, 140, 223, 238, 121, 72, 27, 42,
            193, 240, 163, 146, 5, 52, 103, 86, 120, 73, 26, 43, 188, 141, 222, 239,
            130, 179, 224, 209, 70, 119, 36, 21, 59, 10, 89, 104, 255, 206, 157, 172
        ]
        return table
    
    def _crc8(self, data):
        """Calculate CRC8"""
        crc = 0
        for byte in data:
            crc = self.crc8_table[(crc ^ byte) & 0xFF]
        return crc
    
    def _crc16(self, data):
        """Calculate CRC16"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc
    
    def send_vision_data(self, pitch, yaw, fire_mode=0, target_state=0, target_type=0):
        """
        Send vision data to lower computer
        
        Args:
            pitch: Pitch angle (radians)
            yaw: Yaw angle (radians)
            fire_mode: Fire mode (0=no fire, 1=auto fire, 2=auto aim)
            target_state: Target state (0=no target, 1=converging, 2=ready to fire)
            target_type: Target type (3=Infantry 3)
        """
        # Prepare data
        float_data = [pitch, yaw]
        data_length = 2 + len(float_data) * 4  # flags(2) + floats(8) = 10
        
        # Build flag register
        flags = (fire_mode & 0x03) | \
                ((target_state & 0x03) << 2) | \
                ((target_type & 0x0F) << 4)
        
        # Build frame header
        header = bytearray(4)
        header[0] = 0xA5  # SOF
        header[1] = data_length & 0xFF
        header[2] = (data_length >> 8) & 0xFF
        header[3] = self._crc8(header[0:3])
        
        # Build packet
        packet = bytearray()
        packet.extend(header)
        packet.append(0x01)  # cmd_id = 0x0001 (low byte)
        packet.append(0x00)  # cmd_id (high byte)
        packet.append(flags & 0xFF)  # flags (low byte)
        packet.append((flags >> 8) & 0xFF)  # flags (high byte)
        
        # Add float data (little endian)
        for f in float_data:
            packet.extend(struct.pack('<f', f))
        
        # Calculate and add CRC16
        crc16 = self._crc16(packet)
        packet.append(crc16 & 0xFF)
        packet.append((crc16 >> 8) & 0xFF)
        
        # Send
        self.ser.write(packet)
        self.packet_count += 1
        
        print(f"\n[{self.packet_count:04d}] → TX: pitch={pitch:+.3f}, yaw={yaw:+.3f}, mode={fire_mode}, state={target_state}, type={target_type}")
    
    def receive_attitude_data(self):
        """Receive attitude data from lower computer"""
        if self.ser.in_waiting > 0:
            data = self.ser.read(self.ser.in_waiting)
            
            # Parse binary protocol data only (no text processing)
            for i in range(len(data)):
                if data[i] == 0xA5 and i + 18 <= len(data):
                    packet = data[i:i+18]
                    
                    # Verify header CRC8
                    if self._crc8(packet[0:3]) == packet[3]:
                        # Get data length
                        data_length = packet[1] | (packet[2] << 8)
                        total_length = data_length + 8  # 4(header) + 2(cmd_id) + 2(crc16)
                        
                        if i + total_length <= len(data):
                            full_packet = data[i:i+total_length]
                            
                            # Verify full packet CRC16
                            crc16_calc = self._crc16(full_packet[0:total_length-2])
                            crc16_recv = full_packet[total_length-2] | (full_packet[total_length-1] << 8)
                            
                            if crc16_calc == crc16_recv:
                                # Parse data
                                cmd_id = full_packet[4] | (full_packet[5] << 8)
                                if cmd_id == 0x0002:
                                    yaw = struct.unpack('<f', full_packet[8:12])[0]
                                    pitch = struct.unpack('<f', full_packet[12:16])[0]
                                    roll = struct.unpack('<f', full_packet[16:20])[0]
                                    
                                    print(f"       ← RX: yaw={yaw:+.3f}, pitch={pitch:+.3f}, roll={roll:+.3f}")
                                    return {'yaw': yaw, 'pitch': pitch, 'roll': roll}
        return None
    
    def test_communication(self):
        """
        Test communication by sending 3 vision commands
        """
        print(f"\nStarting communication test (sending 3 commands)...")
        print("=" * 60)
        
        # Test 1: Send target offset right (small angle)
        print("\n[Test 1] Small right offset (yaw=0.05, pitch=0.05)")
        self.send_vision_data(
            pitch=0.05,
            yaw=0.05,
            fire_mode=1,
            target_state=2,  # READY_TO_FIRE
            target_type=3
        )
        
        # Wait and receive response
        for _ in range(5):
            attitude = self.receive_attitude_data()
            time.sleep(0.02)
        time.sleep(0.5)
        
        # Test 2: Send target offset right-up (larger angle)
        print("\n[Test 2] Larger right-up offset (yaw=0.10, pitch=0.10)")
        self.send_vision_data(
            pitch=0.10,
            yaw=0.10,
            fire_mode=1,
            target_state=2,
            target_type=3
        )
        
        # Wait and receive response
        for _ in range(5):
            attitude = self.receive_attitude_data()
            time.sleep(0.02)
        time.sleep(0.5)
        
        # Test 3: Send target offset left-down
        print("\n[Test 3] Left-down offset (yaw=-0.08, pitch=-0.08)")
        self.send_vision_data(
            pitch=-0.08,
            yaw=-0.08,
            fire_mode=2,
            target_state=2,
            target_type=4
        )
        
        # Wait and receive response
        for _ in range(5):
            attitude = self.receive_attitude_data()
            time.sleep(0.02)
        
        print("\n" + "=" * 60)
        print(f"Test complete! Total packets sent: {self.packet_count}\n")
    
    def close(self):
        """Close serial port"""
        self.ser.close()
        print("✓ Serial port closed")

def main():
    print("=" * 60)
    print("  Vision Communication Test Program")
    print("  robomaster-control USB VCP Communication Test")
    print("=" * 60)
    
    # Configure serial port
    # Windows: 'COM3', 'COM4', etc.
    # Linux: '/dev/ttyACM0', '/dev/ttyACM1', etc.
    # macOS: '/dev/cu.usbmodem*'
    
    port = input("\nEnter COM port (Windows: COM3, Linux: /dev/ttyACM0): ").strip()
    if not port:
        port = 'COM3'  # Default value
    
    print(f"\nConnecting to {port}...")
    
    try:
        tester = VisionCommTest(port)
        
        # Run test
        tester.test_communication()
        
        # Close
        tester.close()
        
    except KeyboardInterrupt:
        print("\n\n✓ Test interrupted by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()

