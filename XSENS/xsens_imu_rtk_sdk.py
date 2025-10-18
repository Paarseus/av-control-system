import xsensdeviceapi as xda
import time

# Configuration
PORT = 'COM3'  # Change to your port: Windows: 'COM3', Linux: '/dev/ttyUSB0'
BAUDRATE = 115200

def main():
    print("Connecting to Xsens MT680G...")
    
    # Create XsControl object
    control = xda.XsControl_construct()
    
    if control is None:
        print("Failed to create XsControl object")
        return
    
    try:
        # Scan for connected devices
        portInfoArray = xda.XsScanner_scanPorts()
        
        # Find your device
        mtPort = None
        for i in range(portInfoArray.size()):
            if portInfoArray[i].deviceId().isMti() or portInfoArray[i].deviceId().isMtig():
                mtPort = portInfoArray[i]
                break
        
        if mtPort is None:
            print("No MTi device found. Check connection and port.")
            return
        
        print(f"Found device on {mtPort.portName()}")
        
        # Open port
        if not control.openPort(mtPort.portName(), mtPort.baudrate()):
            print("Could not open port")
            return
        
        # Get device
        device = control.device(mtPort.deviceId())
        
        if device is None:
            print("Could not get device")
            return
        
        print(f"Device: {device.productCode()}, ID: {device.deviceId().toXsString()}")
        
        # Put device in configuration mode
        if not device.gotoConfig():
            print("Could not put device in configuration mode")
            return
        
        # Configure device (optional - if needed)
        # device.setUpdateRate(100)  # Set update rate to 100Hz
        
        # Start measurement
        if not device.gotoMeasurement():
            print("Could not put device in measurement mode")
            return
        
        print("\nReading IMU + RTK GPS data... (Press Ctrl+C to stop)\n")
        
        # Read data loop
        while True:
            if device.isDataAvailable():
                # Get data packet
                packet = device.getDataPacket()
                
                # IMU Data
                if packet.containsOrientation():
                    quat = packet.orientationQuaternion()
                    print(f"Orientation (Quat): w={quat[0]:.4f}, x={quat[1]:.4f}, y={quat[2]:.4f}, z={quat[3]:.4f}")
                
                if packet.containsCalibratedAcceleration():
                    acc = packet.calibratedAcceleration()
                    print(f"Acceleration: x={acc[0]:.3f}, y={acc[1]:.3f}, z={acc[2]:.3f} m/s²")
                
                if packet.containsCalibratedGyroscopeData():
                    gyro = packet.calibratedGyroscopeData()
                    print(f"Gyroscope: x={gyro[0]:.3f}, y={gyro[1]:.3f}, z={gyro[2]:.3f} rad/s")
                
                # RTK GPS Data
                if packet.containsLatitudeLongitude():
                    latlon = packet.latitudeLongitude()
                    print(f"GPS Position: Lat={latlon[0]:.8f}°, Lon={latlon[1]:.8f}°")
                
                if packet.containsAltitude():
                    altitude = packet.altitude()
                    print(f"Altitude: {altitude:.2f} m")
                
                if packet.containsVelocity():
                    vel = packet.velocity()
                    print(f"Velocity: x={vel[0]:.2f}, y={vel[1]:.2f}, z={vel[2]:.2f} m/s")
                
                # GPS Fix Quality
                if packet.containsStatus():
                    status = packet.status()
                    # Check RTK fix status
                    if status & 0x00000004:  # RTK Fixed bit
                        fix_status = "✅ RTK Fixed"
                    elif status & 0x00000002:  # RTK Float bit
                        fix_status = "🟡 RTK Float"
                    else:
                        fix_status = "📍 GPS"
                    print(f"Fix Status: {fix_status}")
                
                print("-" * 60)
                time.sleep(0.1)  # Adjust for desired output rate
            
            time.sleep(0.01)
    
    except KeyboardInterrupt:
        print("\n\nStopping...")
    
    except Exception as e:
        print(f"Error: {e}")
    
    finally:
        # Clean up
        print("Closing device...")
        control.closePort(mtPort.portName())
        control.destruct()
        print("Done.")

if __name__ == "__main__":
    main()
