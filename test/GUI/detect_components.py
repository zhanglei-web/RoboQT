import cv2

# RealSense
try:
    import pyrealsense2 as rs
    HAS_REALSENSE = True
except ImportError:
    HAS_REALSENSE = False

# ZED
try:
    import pyzed.sl as sl
    HAS_ZED = True
except ImportError:
    HAS_ZED = False

# Orbbec SDK
try:
    import pyorbbecsdk as ob
    HAS_ORBBEC = True
except ImportError:
    HAS_ORBBEC = False

# ------------------ 摄像头分配 ------------------
camera_allocation = {}  # { cam_name: comp_type }

def release_camera(cam_name):
    if cam_name in camera_allocation:
        camera_allocation[cam_name] = None

def allocate_camera(cam_name, comp_type):
    if cam_name not in camera_allocation or camera_allocation[cam_name] is None:
        camera_allocation[cam_name] = comp_type
        return True
    return False

# ------------------ 探测器 ------------------
class CameraDetector:
    def __init__(self, max_opencv_index=10):
        self.max_opencv_index = max_opencv_index

    def detect_opencv(self):
        cameras = []
        for i in range(self.max_opencv_index):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                cam_name = f"OpenCV_{i}"
                cameras.append(cam_name)
                if cam_name not in camera_allocation:
                    camera_allocation[cam_name] = None
            cap.release()
        return cameras

    def detect_realsense(self):
        if not HAS_REALSENSE:
            return []
        ctx = rs.context()
        cameras = []
        for dev in ctx.devices:
            name = dev.get_info(rs.camera_info.name)
            serial = dev.get_info(rs.camera_info.serial_number)
            cam_name = f"RealSense_{serial}"
            cameras.append(cam_name)
            if cam_name not in camera_allocation:
                camera_allocation[cam_name] = None
        return cameras

    def detect_orbbec(self):
        if not HAS_ORBBEC:
            return []
        cameras = []
        ctx = ob.Context()
        device_list = ctx.query_devices()
        for i in range(len(device_list)):
            device = device_list.get_device_by_index(i)
            info = device.get_device_info()
            cam_name = f"Orbbec_{info.get_serial_number()}"
            cameras.append(cam_name)
            if cam_name not in camera_allocation:
                camera_allocation[cam_name] = None
        return cameras

    def detect_zed(self):
        if not HAS_ZED:
            return []
        cameras = []
        for info in sl.Camera.get_device_list():
            cam_name = f"ZED_{info.serial_number}"
            cameras.append(cam_name)
            if cam_name not in camera_allocation:
                camera_allocation[cam_name] = None
        return cameras

    def detect_all(self):
        result = {}
        opencv = self.detect_opencv()
        if opencv:
            result["OpenCV"] = {"count": len(opencv), "list": opencv}

        rs_cams = self.detect_realsense()
        if rs_cams:
            result["RealSense"] = {"count": len(rs_cams), "list": rs_cams}

        orbbec = self.detect_orbbec()
        if orbbec:
            result["Orbbec"] = {"count": len(orbbec), "list": orbbec}

        zed = self.detect_zed()
        if zed:
            result["ZED"] = {"count": len(zed), "list": zed}

        return result

# ------------------ 对外接口 ------------------
def get_available_cameras():
    detector = CameraDetector()
    all_cams = detector.detect_all()
    print(all_cams)
    available = []
    for cam_type, info in all_cams.items():
        for cam_name in info["list"]:
            if camera_allocation.get(cam_name) is None: 
                available.append(cam_name)
    return available
