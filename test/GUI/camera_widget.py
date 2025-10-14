# -*- coding: utf-8 -*-
import cv2
from PyQt5.QtWidgets import QWidget, QLabel, QVBoxLayout
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer
import numpy as np
import detect_components

# 如果有 RealSense/ZED/Orbbec SDK 可以在这里导入
try:
    import pyrealsense2 as rs
    HAS_REALSENSE = True
except ImportError:
    HAS_REALSENSE = False

try:
    import pyzed.sl as sl
    HAS_ZED = True
except ImportError:
    HAS_ZED = False

try:
    import pyorbbecsdk as ob
    HAS_ORBBEC = True
except ImportError:
    HAS_ORBBEC = False


class CameraWidget(QWidget):
    def __init__(self, camera_name, parent=None):
        super().__init__(parent)
        self.camera_name = camera_name
        self.cap = None
        self.pipeline = None  # 仅 RealSense 用
        self.zed_cam = None   # 仅 ZED 用
        self.orbbec_cam = None # 仅 Orbbec 用

        self.label = QLabel("正在初始化...", self)
        self.label.setStyleSheet("background-color: black;")
        self.label.setFixedSize(200, 150)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.label)

        # 占用摄像头
        if not detect_components.allocate_camera(camera_name, "GUI_Component"):
            self.label.setText("摄像头已被占用")
            return

        # 初始化摄像头
        self.init_camera()

        # 定时更新
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)  # 约 30fps

    def init_camera(self):
        # ---------------- OpenCV ----------------
        if self.camera_name.startswith("OpenCV_"):
            idx = int(self.camera_name.split("_")[1])
            self.cap = cv2.VideoCapture(idx)
        
        # ---------------- RealSense ----------------
        elif HAS_REALSENSE and self.camera_name.startswith("RealSense_"):
            serial = self.camera_name.split("_")[1]
            ctx = rs.context()
            dev = next((d for d in ctx.devices if d.get_info(rs.camera_info.serial_number) == serial), None)
            if dev:
                self.pipeline = rs.pipeline()
                cfg = rs.config()
                cfg.enable_device(serial)
                cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                self.pipeline.start(cfg)
        
        # ---------------- ZED ----------------
        elif HAS_ZED and self.camera_name.startswith("ZED_"):
            serial = self.camera_name.split("_")[1]
            init_params = sl.InitParameters()
            self.zed_cam = sl.Camera()
            status = self.zed_cam.open(init_params)
            if status != sl.ERROR_CODE.SUCCESS:
                self.label.setText("ZED打开失败")
                self.zed_cam = None
        
        # ---------------- Orbbec ----------------
        elif HAS_ORBBEC and self.camera_name.startswith("Orbbec_"):
            ctx = ob.Context()
            dev_list = ctx.query_devices()
            serial = self.camera_name.split("_")[1]
            self.orbbec_cam = next((dev_list.get_device_by_index(i) for i in range(len(dev_list))
                                    if dev_list.get_device_by_index(i).get_device_info().get_serial_number() == serial), None)
            if self.orbbec_cam is None:
                self.label.setText("Orbbec打开失败")

    def update_frame(self):
        frame = None

        # OpenCV
        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                return
        
        # RealSense
        elif self.pipeline:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if color_frame:
                frame = np.asanyarray(color_frame.get_data())
        
        # ZED
        elif self.zed_cam:
            image = sl.Mat()
            if self.zed_cam.grab() == sl.ERROR_CODE.SUCCESS:
                self.zed_cam.retrieve_image(image, sl.VIEW.LEFT)
                frame = image.get_data()
        
        # Orbbec
        elif self.orbbec_cam:
            frame = self.orbbec_cam.read_color_frame()
        
        if frame is not None:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = frame.shape
            qimg = QImage(frame.data, w, h, ch * w, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qimg)
            self.label.setPixmap(pixmap)

    def closeEvent(self, event):
        # 释放摄像头占用
        detect_components.release_camera(self.camera_name)

        if self.cap and self.cap.isOpened():
            self.cap.release()
        if self.pipeline:
            self.pipeline.stop()
        if self.zed_cam:
            self.zed_cam.close()
        event.accept()
