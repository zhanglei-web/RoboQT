#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import io, sys
import subprocess
import sys
import uuid
import cv2
from PyQt5.QtWidgets import QTextEdit
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
print("所有摄像头:", detect_components.get_available_cameras())
from camera_widget import CameraWidget
from episode_generator import parse_robot_config_to_episode_components, generate_episode_structure

os.chdir(os.path.dirname(os.path.abspath(__file__)))





# 中间画布上单个组件
class ComponentItemWidget(QWidget):
    def __init__(self, comp_type, on_delete, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(10)

        # 创建标签
        self.label = QLabel(comp_type, self)
        layout.addWidget(self.label)

        # 创建删除按钮
        self.delete_button = QPushButton("删除", self)
        self.delete_button.setFixedSize(50, 25)
        self.delete_button.clicked.connect(on_delete)
        layout.addWidget(self.delete_button)

        # 将删除按钮设置在最右边
        layout.setAlignment(self.delete_button, Qt.AlignRight)
        self.setLayout(layout)

# 属性面板
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

        #文件树
        self.episode_display = QTextEdit(self)
        self.episode_display.setReadOnly(True)
        self.episode_display.setMinimumHeight(200)
        self.main_layout.addWidget(QLabel("生成的 Episode 文件树"))
        self.main_layout.addWidget(self.episode_display)


        #弹性空间
        self.main_layout.addStretch()
        
        self.current_sensor_data = None
        self.canvas = None

    def display_episode_tree(self, json_path):
        components = parse_robot_config_to_episode_components(json_path)
        # 不实际创建文件，只获取树状文本

        buf = io.StringIO()
        # 临时重定向 stdout
        old_stdout = sys.stdout
        sys.stdout = buf
        generate_episode_structure(components, create_files=False)
        sys.stdout = old_stdout

        tree_text = buf.getvalue()
        self.episode_display.setText(tree_text)

    def clear_properties(self):
        while self.form_layout.count():
            child = self.form_layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()
        self.current_sensor_data = None
        self.title_label.setText("传感器属性")

    def show_sensor_properties(self, sensor_data):  
        self.clear_properties()  
        self.current_sensor_data = sensor_data  

        # 显示传感器类型  
        self.title_label.setText(f"传感器属性: {sensor_data['type']}")  

        # 显示传感器 ID  
        id_edit = QLineEdit(sensor_data.get('id', ''))  
        id_edit.textChanged.connect(self.update_id)  
        self.form_layout.addRow("ID:", id_edit)  

        
        comp_type = sensor_data["type"]
        comp_info = self.canvas.component_manager.get_flat_components().get(comp_type, {})
        default_params = comp_info.get("default_params", {}) 

        
        params = default_params.copy()
        params.update(sensor_data.get('params', {}))  

        # 显示参数  
        for key, value in params.items():  
            if key == 'output':
                available_outputs = default_params.get("output", [])
                if available_outputs and isinstance(value, list):
                    for option in available_outputs:
                        checkbox = QCheckBox(option)
                        checkbox.setChecked(option in value)
                        checkbox.stateChanged.connect(
                            lambda state, opt=option: self.update_output(opt, state)
                        )
                        self.form_layout.addRow(f"Output {option}:", checkbox)
                else:
                    line_edit = QLineEdit(", ".join(value) if isinstance(value, list) else str(value))
                    line_edit.textChanged.connect(lambda text, k=key: self.update_param(k, text))
                    self.form_layout.addRow("output:", line_edit)

            elif isinstance(value, bool):  
                checkbox = QCheckBox()  
                checkbox.setChecked(value)  
                checkbox.stateChanged.connect(  
                    lambda state, k=key: self.update_param(k, state == Qt.Checked)  
                )  
                self.form_layout.addRow(f"{key}:", checkbox)  

            elif isinstance(value, int):  
                spinbox = QSpinBox()  
                
                spinbox.setRange(0, 99999)
                spinbox.setValue(value)  
                spinbox.valueChanged.connect(  
                    lambda val, k=key: self.update_param(k, val)  
                )  
                self.form_layout.addRow(f"{key}:", spinbox)  

            elif isinstance(value, float):  
                spinbox = QDoubleSpinBox()  
                spinbox.setRange(-9999.0, 9999.0)
                spinbox.setValue(value)  
                spinbox.valueChanged.connect(  
                    lambda val, k=key: self.update_param(k, val)  
                )  
                self.form_layout.addRow(f"{key}:", spinbox)  

            else:  
                line_edit = QLineEdit(str(value))  
                line_edit.textChanged.connect(  
                    lambda text, k=key: self.update_param(k, text)  
                )  
                self.form_layout.addRow(f"{key}:", line_edit)


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

        script_path = "./episode_generator.py"
        try:
            subprocess.run([sys.executable, script_path, save_path], check=True)
            print("[生成 episode] 已调用 episode_generator.py")
        except subprocess.CalledProcessError as e:
            print(f"[错误] 调用 episode_generator.py 失败: {e}")

        # 调用显示函数
        self.display_episode_tree(save_path)

    def apply_config(self):
        if not self.canvas:
            print("Canvas 未设置")
            return

        mode = self.canvas.component_manager.mode
        filename = self.filename_edit.text().strip() or "robot_config"
        json_path = f"../config/{filename}.json"

        current_dir = os.path.dirname(os.path.abspath(__file__))

        if mode == "dora":
            script_path = os.path.join(current_dir, "generate_dora.py")
            try:
                subprocess.run(
                    [sys.executable, script_path, json_path],
                    check=True
                )
                print(f"[应用配置] 已调用 generate_dora.py 处理 {json_path}")
            except subprocess.CalledProcessError as e:
                print(f"[错误] 调用 generate_dora.py 失败: {e}")

        elif mode == "ros":
            script_path = os.path.join(current_dir, "generate_ros.py")
            try:
                subprocess.run(
                    [sys.executable, script_path, json_path],
                    check=True
                )
                print(f"[应用配置] 已调用 generate_ros.py 处理 {json_path}")
            except subprocess.CalledProcessError as e:
                print(f"[错误] 调用 generate_ros.py 失败: {e}")

        else:
            print(f"未知模式: {mode}")

# 中间画布
class RobotCanvas(QGraphicsView):
    def __init__(self, property_panel, component_manager,component_list=None, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        self.property_panel = property_panel
        self.component_manager = component_manager
        self.items_map = []

        self.item_size = (200, 150)
        self.columns = 1
        self.margin = 20

    def show_temporary_message(self, text, duration=2000):
        """显示临时提示文字（自动消失）"""
        label = QLabel(text, self)
        label.setStyleSheet("""
            QLabel {
                background-color: rgba(0, 0, 0, 180);
                color: white;
                padding: 8px 15px;
                border-radius: 8px;
                font-size: 14px;
            }
        """)
        label.adjustSize()
        label.move(
            (self.width() - label.width()) // 2,
            20
        )
        label.show()

        QTimer.singleShot(duration, label.deleteLater)


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
            # self.property_panel.canvas.component_list.refresh_components()

        group, name = comp_type.split("/", 1)
        if group == "camera":
            available_cams = detect_components.get_available_cameras()
            print(f"Available cameras: {available_cams}")  # 调试输出

            matched_cam = available_cams[0] if available_cams else None
            print(f"Selected camera: {matched_cam}")  # 调试输出
            
            if matched_cam:
                success = detect_components.allocate_camera(matched_cam, comp_type)
                if success:
                    widget = CameraWidget(matched_cam, on_delete_callback=on_delete)
                else:
                    widget = QLabel("摄像头被占用")
            else:
                self.show_temporary_message("当前没有可用的摄像头！")
                return
        else:
            widget = ComponentItemWidget(comp_type, on_delete)


        proxy = QGraphicsProxyWidget()
        proxy.setWidget(widget)
        proxy.setFlag(QGraphicsProxyWidget.ItemIsSelectable, True)
        proxy.setZValue(10)

        data = {
            "type": comp_type,
            "params": default_params.copy(),
            "id": str(uuid.uuid4())
        }
        proxy.sensor_data = data
        self.items_map.append({"proxy": proxy, "data": data})
        self._scene.addItem(proxy)
        self.relayout_items()
        # self.property_panel.canvas.component_list.refresh_components()

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

# 左侧组件库 
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

# 主窗口
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
            "../config/components.yaml"
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
