import math
import os
import cv2
import depthai as dai
import numpy as np
from datetime import timedelta
from dotenv import load_dotenv

load_dotenv()

# User-defined constants
DEVICE_IP = os.environ.get("OAK_DEVICE_IP", "192.168.1.100")
CRITICAL = 2000  # in mm, initial value (will be controlled by slider)

fontType = cv2.FONT_HERSHEY_TRIPLEX


# Slider callback function
def on_critical_distance_change(val):
    global CRITICAL
    # Convert slider value (in 0.5m increments) to mm
    CRITICAL = val * 500  # val ranges from 1 to 10, representing 0.5m to 5m


# Create window and slider before starting camera
window_name = "Collision Avoidance"
cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
cv2.resizeWindow(window_name, 1280, 800)
# Slider range: 1 to 10 (representing 0.5m to 5m in 0.5m increments)
# Initial value: 4 (representing 2m)
cv2.createTrackbar(
    "Critical Distance (0.5x meter)",
    window_name,
    4,
    10,
    on_critical_distance_change,
)

# Create device and pipeline
device_info = dai.DeviceInfo(DEVICE_IP)
device = dai.Device(device_info)
pipeline = dai.Pipeline(device)

# Define sources - RGB and stereo depth cameras (v3 Camera nodes)
camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

# Create stereo depth node
stereo = pipeline.create(dai.node.StereoDepth)
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)
stereo.setLeftRightCheck(True)
stereo.setExtendedDisparity(True)
stereo.setSubpixel(False)
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)  # Align depth to RGB
stereo.setOutputSize(1280, 720)  # Width must be multiple of 16

# Link mono cameras to stereo (width must be multiple of 16 for StereoDepth)
monoLeft.requestOutput((640, 400)).link(stereo.left)
monoRight.requestOutput((640, 400)).link(stereo.right)

# Spatial location calculator configuration
slc = pipeline.create(dai.node.SpatialLocationCalculator)
for x in range(15):
    for y in range(9):
        config = dai.SpatialLocationCalculatorConfigData()
        config.depthThresholds.lowerThreshold = 200
        config.depthThresholds.upperThreshold = 10000
        # ROI width x height = 80 x 72 pixels.
        config.roi = dai.Rect(
            dai.Point2f((x + 0.5) * 0.0625, (y + 0.5) * 0.1),
            dai.Point2f((x + 1.5) * 0.0625, (y + 1.5) * 0.1),
        )
        config.calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MEDIAN
        slc.initialConfig.addROI(config)

stereo.depth.link(slc.inputDepth)

# Create message sync node to synchronize RGB and depth
sync = pipeline.create(dai.node.Sync)
sync.setSyncThreshold(timedelta(milliseconds=20))

camRgb.requestOutput((1280, 720)).link(sync.inputs["rgb"])
stereo.depth.link(sync.inputs["depth"])

# Create output queues directly from node outputs (no XLinkOut needed in v3)
qSync = sync.out.createOutputQueue()
qSlc = slc.out.createOutputQueue()

# Start pipeline
pipeline.start()

print("Started!")

while pipeline.isRunning():
    messageGroup = qSync.get()

    if messageGroup is None:
        continue

    rgbFrame = messageGroup["rgb"].getCvFrame()
    depthFrame = messageGroup["depth"].getFrame()

    # Get spatial location data
    slcData = qSlc.tryGet()
    if slcData is not None:
        slc_data = slcData.getSpatialLocations()
    else:
        slc_data = []

    # Colorize depth for visualization (normalize to 10m max range)
    depthFrameColor = np.clip(depthFrame, 0, 10000)
    depthFrameColor = (depthFrameColor * 255.0 / 10000).astype(np.uint8)
    depthColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_BONE)

    # Blend RGB and depth for better visualization
    blendedFrame = cv2.addWeighted(rgbFrame, 0.6, depthColor, 0.4, 0)

    # Draw collision warnings on the blended frame
    for depthData in slc_data:
        roi = depthData.config.roi
        roi = roi.denormalize(
            width=blendedFrame.shape[1], height=blendedFrame.shape[0]
        )

        xmin = int(roi.topLeft().x)
        ymin = int(roi.topLeft().y)
        xmax = int(roi.bottomRight().x)
        ymax = int(roi.bottomRight().y)

        coords = depthData.spatialCoordinates
        distance = math.sqrt(coords.x**2 + coords.y**2 + coords.z**2)

        if distance == 0:  # Invalid
            continue

        # Only show CRITICAL level
        if distance < CRITICAL:
            color = (0, 0, 255)
            cv2.rectangle(
                blendedFrame,
                (xmin, ymin),
                (xmax, ymax),
                color,
                thickness=3,
            )
            cv2.putText(
                blendedFrame,
                "{:.1f}m".format(distance / 1000),
                (xmin + 5, ymin + 20),
                fontType,
                0.5,
                color,
                1,
            )

    # Display current critical distance on the frame
    slider_val = cv2.getTrackbarPos(
        "Critical Distance (0.5x meter)", window_name
    )
    current_distance = slider_val * 0.5
    cv2.putText(
        blendedFrame,
        f"Critical Distance: {current_distance:.1f}m",
        (10, 700),
        fontType,
        0.7,
        (0, 255, 255),
        2,
        cv2.LINE_AA
    )

    cv2.imshow(window_name, blendedFrame)

    if cv2.waitKey(1) == ord("q"):
        break

cv2.destroyAllWindows()
