#!/usr/bin/env python3
"""
ESP32S3 - Crazyflie Connection Monitor
This script monitors the connection between a Xiao Sense ESP32S3 and Crazyflie
through the breakout deck using jumper cables (3V3 to VCC, GND to GND).
"""

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie import Crazyflie
import time
import threading
import logging
from datetime import datetime
import serial
import serial.tools.list_ports

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('connection_log.txt'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

class ConnectionMonitor:
    def __init__(self):
        self.crazyflie = None
        self.esp32_serial = None
        self.connection_status = {
            'crazyflie_connected': False,
            'esp32_connected': False,
            'communication_active': False
        }
        self.monitoring = False
        
    def initialize_crazyflie_drivers(self):
        """Initialize Crazyflie drivers and scan for devices"""
        try:
            logger.info("Initializing Crazyflie drivers...")
            cflib.crtp.init_drivers(enable_debug_driver=False)
            
            # Scan for Crazyflies
            available = cflib.crtp.scan_interfaces()
            logger.info(f"Found {len(available)} Crazyflie device(s)")
            
            for uri in available:
                logger.info(f"  - {uri}")
                
            return available
        except Exception as e:
            logger.error(f"Failed to initialize Crazyflie drivers: {e}")
            return []
    
    def scan_esp32_ports(self):
        """Scan for ESP32 serial ports"""
        try:
            ports = list(serial.tools.list_ports.comports())
            esp32_ports = []
            
            logger.info("Scanning for ESP32 serial ports...")
            for port in ports:
                logger.info(f"  - {port.device}: {port.description}")
                # Look for ESP32-related ports
                if any(keyword in port.description.lower() for keyword in ['esp32', 'xiao', 'ch340', 'cp210']):
                    esp32_ports.append(port.device)
            
            return esp32_ports
        except Exception as e:
            logger.error(f"Failed to scan serial ports: {e}")
            return []
    
    def connect_to_crazyflie(self, uri):
        """Connect to Crazyflie and monitor connection"""
        try:
            logger.info(f"Attempting to connect to Crazyflie at {uri}")
            
            # Create connection with callbacks
            cf = Crazyflie()
            
            # Set up connection callbacks
            cf.connected.add_callback(self._crazyflie_connected)
            cf.disconnected.add_callback(self._crazyflie_disconnected)
            cf.connection_failed.add_callback(self._crazyflie_connection_failed)
            
            # Connect
            cf.open_link(uri)
            
            # Wait for connection
            timeout = 10
            start_time = time.time()
            while not self.connection_status['crazyflie_connected'] and (time.time() - start_time) < timeout:
                time.sleep(0.1)
            
            if self.connection_status['crazyflie_connected']:
                self.crazyflie = cf
                logger.info("✓ Crazyflie connected successfully!")
                return True
            else:
                logger.error("✗ Crazyflie connection timeout")
                return False
                
        except Exception as e:
            logger.error(f"✗ Failed to connect to Crazyflie: {e}")
            return False
    
    def connect_to_esp32(self, port, baudrate=115200):
        """Connect to ESP32 via serial"""
        try:
            logger.info(f"Attempting to connect to ESP32 at {port} (baudrate: {baudrate})")
            
            self.esp32_serial = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=1,
                write_timeout=1
            )
            
            # Test communication
            self.esp32_serial.write(b"PING\n")
            response = self.esp32_serial.readline().decode().strip()
            
            if response:
                logger.info(f"✓ ESP32 responded: {response}")
                self.connection_status['esp32_connected'] = True
                return True
            else:
                logger.warning("⚠ ESP32 connected but no response to ping")
                self.connection_status['esp32_connected'] = True
                return True
                
        except Exception as e:
            logger.error(f"✗ Failed to connect to ESP32: {e}")
            return False
    
    def _crazyflie_connected(self, uri):
        """Callback when Crazyflie connects"""
        logger.info(f"✓ Crazyflie connected: {uri}")
        self.connection_status['crazyflie_connected'] = True
        
        # Try to read some basic parameters
        try:
            if self.crazyflie:
                # Read firmware version
                self.crazyflie.param.get_value("firmware.revision0").add_callback(self._firmware_version_callback)
                
                # Read battery voltage
                self.crazyflie.param.get_value("pm.vbat").add_callback(self._battery_voltage_callback)
                
        except Exception as e:
            logger.warning(f"Could not read Crazyflie parameters: {e}")
    
    def _crazyflie_disconnected(self, uri):
        """Callback when Crazyflie disconnects"""
        logger.warning(f"⚠ Crazyflie disconnected: {uri}")
        self.connection_status['crazyflie_connected'] = False
    
    def _crazyflie_connection_failed(self, uri, msg):
        """Callback when Crazyflie connection fails"""
        logger.error(f"✗ Crazyflie connection failed: {uri} - {msg}")
        self.connection_status['crazyflie_connected'] = False
    
    def _firmware_version_callback(self, name, value):
        """Callback for firmware version"""
        logger.info(f"Firmware version: {value}")
    
    def _battery_voltage_callback(self, name, value):
        """Callback for battery voltage"""
        logger.info(f"Battery voltage: {value}V")
    
    def monitor_connections(self):
        """Continuously monitor both connections"""
        self.monitoring = True
        logger.info("Starting connection monitoring...")
        
        while self.monitoring:
            status_msg = []
            
            if self.connection_status['crazyflie_connected']:
                status_msg.append("Crazyflie: ✓")
            else:
                status_msg.append("Crazyflie: ✗")
            
            if self.connection_status['esp32_connected']:
                status_msg.append("ESP32: ✓")
            else:
                status_msg.append("ESP32: ✗")
            
            logger.info(f"Status: {' | '.join(status_msg)}")
            
            # Check for data from ESP32
            if self.esp32_serial and self.esp32_serial.in_waiting:
                try:
                    data = self.esp32_serial.readline().decode().strip()
                    if data:
                        logger.info(f"ESP32 data: {data}")
                except Exception as e:
                    logger.warning(f"Error reading ESP32 data: {e}")
            
            time.sleep(2)
    
    def test_power_connection(self):
        """Test the power connection between ESP32 and Crazyflie"""
        logger.info("Testing power connection...")
        
        if not self.connection_status['crazyflie_connected']:
            logger.error("Cannot test power connection - Crazyflie not connected")
            return False
        
        if not self.connection_status['esp32_connected']:
            logger.error("Cannot test power connection - ESP32 not connected")
            return False
        
        try:
            # Try to read battery voltage before and after ESP32 connection
            logger.info("Testing power draw...")
            
            # Send a command to ESP32 to draw some power
            if self.esp32_serial:
                self.esp32_serial.write(b"POWER_TEST\n")
                time.sleep(1)
                
                # Read battery voltage
                if self.crazyflie:
                    self.crazyflie.param.get_value("pm.vbat").add_callback(self._power_test_callback)
            
            return True
            
        except Exception as e:
            logger.error(f"Power test failed: {e}")
            return False
    
    def _power_test_callback(self, name, value):
        """Callback for power test"""
        logger.info(f"Power test - Battery voltage: {value}V")
    
    def send_esp32_command(self, command):
        """Send a command to ESP32"""
        if self.esp32_serial and self.connection_status['esp32_connected']:
            try:
                self.esp32_serial.write(f"{command}\n".encode())
                logger.info(f"Sent command to ESP32: {command}")
            except Exception as e:
                logger.error(f"Failed to send command to ESP32: {e}")
        else:
            logger.error("ESP32 not connected")
    
    def cleanup(self):
        """Clean up connections"""
        logger.info("Cleaning up connections...")
        self.monitoring = False
        
        if self.crazyflie:
            try:
                self.crazyflie.close_link()
                logger.info("Crazyflie connection closed")
            except:
                pass
        
        if self.esp32_serial:
            try:
                self.esp32_serial.close()
                logger.info("ESP32 serial connection closed")
            except:
                pass

def main():
    """Main function to run the connection monitor"""
    print("=" * 60)
    print("ESP32S3 - Crazyflie Connection Monitor")
    print("=" * 60)
    print("This script will help you debug the connection between")
    print("your Xiao Sense ESP32S3 and Crazyflie via jumper cables.")
    print("Make sure you have connected:")
    print("  - 3V3 from ESP32 to VCC on Crazyflie breakout")
    print("  - GND from ESP32 to GND on Crazyflie breakout")
    print("=" * 60)
    
    monitor = ConnectionMonitor()
    
    try:
        # Initialize and scan for devices
        crazyflie_devices = monitor.initialize_crazyflie_drivers()
        esp32_ports = monitor.scan_esp32_ports()
        
        if not crazyflie_devices:
            logger.error("No Crazyflie devices found!")
            print("\nTroubleshooting:")
            print("1. Make sure Crazyflie is powered on")
            print("2. Check USB connection")
            print("3. Try different USB port")
            return
        
        if not esp32_ports:
            logger.warning("No ESP32 serial ports found!")
            print("\nTroubleshooting:")
            print("1. Make sure ESP32 is connected via USB")
            print("2. Check if ESP32 is powered on")
            print("3. Try different USB cable")
        
        # Connect to Crazyflie
        if crazyflie_devices:
            uri = crazyflie_devices[0][0]
            if monitor.connect_to_crazyflie(uri):
                logger.info("Crazyflie connection successful!")
            else:
                logger.error("Failed to connect to Crazyflie")
                return
        
        # Connect to ESP32
        if esp32_ports:
            if monitor.connect_to_esp32(esp32_ports[0]):
                logger.info("ESP32 connection successful!")
            else:
                logger.warning("Failed to connect to ESP32")
        
        # Start monitoring
        print("\nStarting connection monitoring...")
        print("Press Ctrl+C to stop")
        print("-" * 60)
        
        # Start monitoring in a separate thread
        monitor_thread = threading.Thread(target=monitor.monitor_connections)
        monitor_thread.daemon = True
        monitor_thread.start()
        
        # Interactive commands
        while True:
            try:
                command = input("\nEnter command (help, test, power, quit): ").strip().lower()
                
                if command == 'quit':
                    break
                elif command == 'help':
                    print("Available commands:")
                    print("  help   - Show this help")
                    print("  test   - Test power connection")
                    print("  power  - Send power test to ESP32")
                    print("  quit   - Exit program")
                elif command == 'test':
                    monitor.test_power_connection()
                elif command == 'power':
                    monitor.send_esp32_command("POWER_TEST")
                else:
                    print("Unknown command. Type 'help' for available commands.")
                    
            except KeyboardInterrupt:
                break
            except EOFError:
                break
    
    except KeyboardInterrupt:
        print("\nStopping monitor...")
    finally:
        monitor.cleanup()
        print("Monitor stopped.")

if __name__ == "__main__":
    main()
