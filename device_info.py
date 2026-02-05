import os
import depthai as dai
from dotenv import load_dotenv

load_dotenv()

# Device IP address
DEVICE_IP = os.environ.get("OAK_DEVICE_IP", "192.168.1.100")

# Connect to device
device_info = dai.DeviceInfo(DEVICE_IP)
with dai.Device(device_info) as device:
    print("=" * 60)
    print("DEVICE INFORMATION")
    print("=" * 60)

    # Basic device info
    print(f"Device Name: {device.getDeviceName()}")
    print(f"MxId: {device.getDeviceId()}")
    print(f"USB Speed: {device.getUsbSpeed()}")

    # Get connected cameras
    print("\n" + "=" * 60)
    print("CONNECTED CAMERAS")
    print("=" * 60)

    connected_cameras = device.getConnectedCameras()
    print(f"Number of cameras: {len(connected_cameras)}")

    for cam_socket in connected_cameras:
        print(f"\n  Camera Socket: {cam_socket}")

        # Get camera features
        features = device.getCameraSensorNames()
        if cam_socket in features:
            print(f"  Sensor Name: {features[cam_socket]}")

        # Get camera intrinsics if available
        calib = device.readCalibration()
        try:
            intrinsics = calib.getCameraIntrinsics(cam_socket)
            print(f"  Intrinsics: {intrinsics}")
        except:
            print(f"  Intrinsics: Not available")

    # Board socket names mapping
    print("\n" + "=" * 60)
    print("CAMERA SOCKET MAPPING")
    print("=" * 60)
    print("  CAM_A: RGB camera (usually)")
    print("  CAM_B: Left mono camera (usually)")
    print("  CAM_C: Right mono camera (usually)")
    print("  CAM_D: Additional camera (if available)")

    # Get available resolutions
    print("\n" + "=" * 60)
    print("SENSOR DETAILS")
    print("=" * 60)

    sensors = device.getCameraSensorNames()
    for socket, sensor_name in sensors.items():
        print(f"\n  Socket {socket}:")
        print(f"    Sensor: {sensor_name}")

    # Calibration data
    print("\n" + "=" * 60)
    print("CALIBRATION DATA")
    print("=" * 60)

    calib = device.readCalibration()
    print(f"  Baseline distance (stereo): {calib.getBaselineDistance():.2f} cm")

    # Get stereo camera sockets
    try:
        left_socket = calib.getStereoLeftCameraId()
        right_socket = calib.getStereoRightCameraId()
        print(f"  Stereo Left Camera: {left_socket}")
        print(f"  Stereo Right Camera: {right_socket}")
    except:
        print("  Stereo calibration: Not available")

    # Device temperature and other info
    print("\n" + "=" * 60)
    print("ADDITIONAL INFO")
    print("=" * 60)
    print(f"  Boot from flash: {device.getBootloaderVersion()}")

    print("\n" + "=" * 60)

# Find all connected devices
deviceInfos = dai.DeviceBootloader.getAllAvailableDevices()

if len(deviceInfos) == 0:
    print("No devices found!")
    exit(-1)

print("Found devices:")
for i, di in enumerate(deviceInfos):
    print(f'\n[{i}] {di.getDeviceId()} [{di.protocol.name}]')
    
    # Connect to bootloader
    if di.state == dai.XLinkDeviceState.X_LINK_BOOTLOADER:
        with dai.DeviceBootloader(di) as bl:
            version = bl.getVersion()
            print(f"  Current bootloader version: {version}")
            
            # Check if device supports user bootloader (dual bootloader system)
            if hasattr(bl, 'isUserBootloaderSupported'):
                supported = bl.isUserBootloaderSupported()
                print(f"  User bootloader supported: {supported}")
                
                if supported:
                    is_user = bl.isUserBootloader()
                    print(f"  Currently running: {'User bootloader' if is_user else 'Factory bootloader'}")
                    
                    # Try to read config to check for user bootloader size
                    try:
                        config = bl.readConfigData()
                        if 'userBlSize' in config:
                            user_bl_size = config.get('userBlSize', 0)
                            print(f"  User bootloader size: {user_bl_size} bytes")
                            if user_bl_size > 0:
                                print("  ✓ Device HAS factory bootloader fallback protection!")
                            else:
                                print("  ✓ Device supports user bootloader but none is flashed yet")
                    except Exception as e:
                        print(f"  Could not read config: {e}")
                else:
                    print("  ✗ Device does NOT have factory bootloader fallback")
                    print("    (manufactured before 2023 or bootloader < 0.0.26)")
            else:
                print("  Unable to check (depthai library too old)")
    else:
        print("  Device not in bootloader mode")
