#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
import uuid
import cv2
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QGraphicsView, QGraphicsScene, QGraphicsProxyWidget,
    QFrame, QFormLayout, QSpinBox, QDoubleSpinBox, QTextEdit,
    QLineEdit, QCheckBox, QComboBox
)
from PyQt5.QtGui import QImage, QPixmap
import numpy as np
from component_manager import ComponentManager
from yaml_to_json import generate_config
import detect_components


def generate_dora_config(*args, **kwargs):
    print("[占位] generate_dora_config 调用")

def generate_ros_config(*args, **kwargs):
    print("[占位] generate_ros_config 调用")

# ------------------ 摄像头显示部件 ------------------
class CameraWidget(QWidget):
    def __init__(self, camera_name, parent=None):
        super().__init__(parent)
        self.camera_name = camera_name
        self.label = QLabel("正在初始化...", self)
        self.label.setStyleSheet("background-color: black;")
        self.label.setFixedSize(200, 150)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0,0,0,0)
        layout.addWidget(self.label)

        self.cap = None
        self.init_camera()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)  # 大约 30fps

    def init_camera(self):
        if self.camera_name.startswith("OpenCV_"):
            idx = int(self.camera_name.split("_")[1])
            self.cap = cv2.VideoCapture(idx)
        # 根据摄像头类型增加 RealSense/ZED/Orbbec 支持

    def update_frame(self):
        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = frame.shape
                qimg = QImage(frame.data, w, h, ch * w, QImage.Format_RGB888)
                pixmap = QPixmap.fromImage(qimg)
                self.label.setPixmap(pixmap)

    def closeEvent(self, event):
        if self.cap and self.cap.isOpened():
            self.cap.release()
        event.accept()

# ------------------ 中间画布上单个组件 ------------------
class ComponentItemWidget(QWidget):
    def __init__(self, comp_type, on_delete, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(5, 2, 5, 2)
        layout.setSpacing(5)

        self.label = QLabel(comp_type, self)
        layout.addWidget(self.label)

        self.delete_button = QPushButton("删除", self)
        self.delete_button.setFixedSize(50, 25)
        self.delete_button.clicked.connect(on_delete)
        layout.addWidget(self.delete_button)

# ------------------ 属性面板 ------------------
class PropertyPanel(QWidget):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setMaximumWidth(350)
        self.setMinimumWidth(300)

        #初始化垂直布局
        self.main_layout = QVBoxLayout(self)
        
        #加一个标签
        self.title_label = QLabel("传感器属性", self)
        self.main_layout.addWidget(self.title_label)

        #加一个表单布局
        self.form_layout = QFormLayout()
        self.main_layout.addLayout(self.form_layout)

        #加一个标签
        self.info_label = QLabel("请点击画布中的组件查看属性", self)
        self.info_label.setWordWrap(True)#允许文本换行
        self.main_layout.addWidget(self.info_label)

        #添加一个标签
        self.main_layout.addWidget(QLabel("机器人名称"))
        #创建一个文本输入框
        self.filename_edit = QLineEdit("robot_config")
        self.main_layout.addWidget(self.filename_edit)
        #创建一个复选框
        self.use_videos_checkbox = QCheckBox("use_videos")
        self.main_layout.addWidget(self.use_videos_checkbox)

        #创建一个按钮，绑定按钮点击事件
        self.save_button = QPushButton("保存配置")
        self.save_button.clicked.connect(self.save_config)
        self.main_layout.addWidget(self.save_button)

        self.apply_button = QPushButton("应用配置")
        self.apply_button.clicked.connect(self.apply_config)
        self.main_layout.addWidget(self.apply_button)

        #弹性空间
        self.main_layout.addStretch()
        
        self.current_sensor_data = None
        self.canvas = None

    #清除所有动态生成的表单控件
    def clear_properties(self):
        while self.form_layout.count():
            child = self.form_layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()
        self.current_sensor_data = None
        self.title_label.setText("传感器属性")


    def update_output(self, option, state):
        if not self.current_sensor_data:
            return
        outputs = self.current_sensor_data["params"].get("output", [])
        if state == Qt.Checked:
            if option not in outputs:
                outputs.append(option)
        else:
            if option in outputs:
                outputs.remove(option)
        self.current_sensor_data["params"]["output"] = outputs

    def update_param(self, key, value):
        if self.current_sensor_data:
            self.current_sensor_data["params"][key] = value

    def update_id(self, value):
        if self.current_sensor_data:
            self.current_sensor_data["id"] = value

    def save_config(self):
        if not self.canvas:
            print("Canvas 未设置")
            return
        components_data = self.canvas.items_map
        use_videos = self.use_videos_checkbox.isChecked()
        filename = self.filename_edit.text().strip() or "robot_config"
        save_path = generate_config(
            components=components_data,
            component_manager=self.canvas.component_manager,
            use_videos=use_videos,
            filename=filename
        )
        print(f"[保存配置] 已生成配置文件: {save_path}")

    def apply_config(self):
        if not self.canvas:
            print("Canvas 未设置")
            return
        mode = self.canvas.component_manager.mode
        filename = self.filename_edit.text().strip() or "robot_config"
        if mode == "dora":
            generate_dora_config(None, filename)
            print(f"[应用配置] 调用了 generate_dora_config 应用 {filename}.json")
        elif mode == "ros":
            generate_ros_config(None, filename)
            print(f"[应用配置] 调用了 generate_ros_config 应用 {filename}.json")
        else:
            print(f"未知模式: {mode}")

# ------------------ 中间画布 ------------------
class RobotCanvas(QGraphicsView):
    def __init__(self, property_panel, component_manager, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        self.property_panel = property_panel
        self.component_manager = component_manager
        self.items_map = []

        self.item_size = (200, 150)
        self.columns = 3
        self.margin = 20

    def add_component(self, comp_type):
        comp_info = self.component_manager.get_flat_components()[comp_type]
        default_params = comp_info.get("default_params", {})

        def on_delete():
            if group == "camera" and hasattr(widget, "camera_name"):
                detect_components.release_camera(widget.camera_name)
            self._scene.removeItem(proxy)
            self.items_map = [it for it in self.items_map if it["proxy"] != proxy]
            self.property_panel.clear_properties()
            self.relayout_items()

        group, name = comp_type.split("/", 1)
        if group == "camera":
            available_cams = detect_components.get_available_cameras()
            print(available_cams)
            matched_cam = None
            for cam in available_cams:
                if name.lower() in cam.lower():
                    matched_cam = cam
                    break
            if not matched_cam and available_cams:
                matched_cam = available_cams[0]
            if matched_cam:
                success = detect_components.allocate_camera(matched_cam, comp_type)
                if success:
                    widget = CameraWidget(matched_cam)
                else:
                    widget = QLabel("摄像头被占用")
            else:
                widget = QLabel("无可用相机")
        else:
            widget = ComponentItemWidget(comp_type, on_delete)

        proxy = QGraphicsProxyWidget()
        proxy.setWidget(widget)
        proxy.setFlag(QGraphicsProxyWidget.ItemIsSelectable, True)

        data = {
            "type": comp_type,
            "params": default_params.copy(),
            "id": str(uuid.uuid4())
        }
        proxy.sensor_data = data
        self.items_map.append({"proxy": proxy, "data": data})
        self._scene.addItem(proxy)
        self.relayout_items()

    def relayout_items(self):
        for idx, item in enumerate(self.items_map):
            row = idx // self.columns
            col = idx % self.columns
            x = col * (self.item_size[0] + self.margin)
            y = row * (self.item_size[1] + self.margin)
            item["proxy"].setPos(x, y)

    def mousePressEvent(self, event):
        super().mousePressEvent(event)
        item = self.itemAt(event.pos())
        if isinstance(item, QGraphicsProxyWidget) and hasattr(item, "sensor_data"):
            self.property_panel.show_sensor_properties(item.sensor_data)

# ------------------ 左侧组件库 ------------------
class ComponentListPanel(QWidget):
    def __init__(self, component_manager, canvas, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.component_manager = component_manager
        self.canvas = canvas

        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(5, 5, 5, 5)
        self.layout.setSpacing(10)
        self.setMaximumWidth(250)

        self.mode_selector = QComboBox(self)
        self.mode_selector.addItems(["dora", "ros"])
        self.mode_selector.currentTextChanged.connect(self.on_mode_changed)
        self.layout.addWidget(QLabel("选择模式"))
        self.layout.addWidget(self.mode_selector)

        self.items_container = QVBoxLayout()
        self.layout.addLayout(self.items_container)
        self.layout.addStretch()

        self.refresh_components()

    def on_mode_changed(self, mode):
        self.component_manager.set_mode(mode)
        self.refresh_components()

    def refresh_components(self):
        while self.items_container.count():
            child = self.items_container.takeAt(0)
            if child.widget():
                child.widget().deleteLater()

        components = self.component_manager.get_flat_components()
        available_cams = detect_components.get_available_cameras()

        for comp_type, comp_info in components.items():
            group, name = comp_type.split("/", 1)
            if group == "camera" and not any(name.lower() in cam.lower() for cam in available_cams):
                continue
            item_widget = QFrame(self)
            item_layout = QHBoxLayout(item_widget)
            name_label = QLabel(comp_type, item_widget)
            item_layout.addWidget(name_label)
            add_button = QPushButton("+", item_widget)
            add_button.setFixedSize(30, 30)
            add_button.clicked.connect(lambda _, t=comp_type: self.canvas.add_component(t))
            item_layout.addWidget(add_button)
            self.items_container.addWidget(item_widget)

# ------------------ 主窗口 ------------------
class RobotConfigWindow(QWidget):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        #设置画布大小及标题
        self.resize(1300, 800)
        self.setWindowTitle("机器人传感器配置工具")
        #设置主要布局
        layout = QHBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        
        #初始化组件管理器
        self.component_manager = ComponentManager(
            "/home/ray/WanX-EI-Studio/operating_platform/robot/RoboQT/test/config/components.yaml"
        )

        #创建三大核心组件
        #属性面板
        self.property_panel = PropertyPanel(self)
        #中间画布
        self.canvas = RobotCanvas(self.property_panel, self.component_manager, self)
        self.property_panel.canvas = self.canvas
        #组件列表
        self.component_list = ComponentListPanel(self.component_manager, self.canvas, self)

        layout.addWidget(self.component_list)
        layout.addWidget(self.canvas)
        layout.addWidget(self.property_panel)

# ------------------ 启动 ------------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RobotConfigWindow()
    window.show()
    sys.exit(app.exec_())
