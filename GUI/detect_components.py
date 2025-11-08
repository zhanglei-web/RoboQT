import cv2

# realsense
try:
    import pyrealsense2 as rs
    HAS_realsense = True
except ImportError:
    HAS_realsense = False

# zed
try:
    import pyzed.sl as sl
    HAS_zed = True
except ImportError:
    HAS_zed = False

# orbbec SDK
try:
    import pyorbbecsdk as ob
    HAS_orbbec = True
except ImportError:
    HAS_orbbec = False

# ------------------ 摄像头分配 ------------------
# { cam_name: comp_type }
camera_allocation = {}  

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
                cam_name = f"opencv_{i}"
                cameras.append(cam_name)
                if cam_name not in camera_allocation:
                    camera_allocation[cam_name] = None
            cap.release()
        return cameras

    def detect_realsense(self):  
        if not HAS_realsense:  
            return []  
        ctx = rs.context()  
        cameras = []  
        device_list = ctx.query_devices() 
        for dev in device_list:  
            name = dev.get_info(rs.camera_info.name)  
            serial = dev.get_info(rs.camera_info.serial_number)  
            cam_name = f"realsense_{serial}"  
            cameras.append(cam_name)  
            if cam_name not in camera_allocation:  
                camera_allocation[cam_name] = None  
        return cameras

    def detect_orbbec(self):  
        if not HAS_orbbec:  
            return []  
        cameras = []  
        ctx = ob.Context()  
        device_list = ctx.query_devices()  
        for device in device_list:  
            info = device.get_device_info()  
            cam_name = f"orbbec_{info.get_serial_number()}"  
            cameras.append(cam_name)  
            if cam_name not in camera_allocation:  
                camera_allocation[cam_name] = None  
        return cameras

    def detect_zed(self):
        if not HAS_zed:
            return []
        cameras = []
        for info in sl.Camera.get_device_list():
            cam_name = f"zed_{info.serial_number}"
            cameras.append(cam_name)
            if cam_name not in camera_allocation:
                camera_allocation[cam_name] = None
        return cameras

    def detect_all(self):
        result = {}
        opencv = self.detect_opencv()
        if opencv:
            result["opencv"] = {"count": len(opencv), "list": opencv}

        rs_cams = self.detect_realsense()
        if rs_cams:
            result["realsense"] = {"count": len(rs_cams), "list": rs_cams}

        orbbec = self.detect_orbbec()
        if orbbec:
            result["orbbec"] = {"count": len(orbbec), "list": orbbec}

        zed = self.detect_zed()
        if zed:
            result["zed"] = {"count": len(zed), "list": zed}

        return result

# ------------------ 对外接口 ------------------
def get_available_cameras():
    detector = CameraDetector()
    all_cams = detector.detect_all()
    #print(all_cams)
    available = []
    for cam_type, info in all_cams.items():
        for cam_name in info["list"]:
            if camera_allocation.get(cam_name) is None: 
                available.append(cam_name)
    return available
