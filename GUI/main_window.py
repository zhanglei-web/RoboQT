#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import os
import yaml
import json
import uuid
import cgitb
import subprocess
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QGraphicsView, QGraphicsScene, QGraphicsProxyWidget,
    QFrame, QFormLayout, QSpinBox, QDoubleSpinBox, QTextEdit, QLineEdit, QCheckBox
)


os.chdir(os.path.dirname(os.path.abspath(__file__)))

# ------------------ 组件管理器 ------------------
class ComponentManager:
    def __init__(self, yaml_file="../components.yaml"):
        self.yaml_file = yaml_file
        self.components = self.load_components()
        self.flat_components = self.flatten_components(self.components)

    def load_components(self):
        if os.path.exists(self.yaml_file):
            with open(self.yaml_file, "r", encoding="utf-8") as f:
                return yaml.safe_load(f)
        else:
            raise FileNotFoundError(f"未找到配置文件: {self.yaml_file}")

    def flatten_components(self, data, prefix=""):
        """
        把多层嵌套的 dora/camera/camera_opencv 结构打平成:
        {
          "dora/camera/camera_opencv": {...},
          "dora/arm/robot_arm": {...}
        }
        """
        flat = {}
        for key, value in data.items():
            if isinstance(value, dict) and "default_params" not in value:
                # 不是最终组件，继续递归
                new_prefix = f"{prefix}/{key}" if prefix else key
                flat.update(self.flatten_components(value, new_prefix))
            else:
                # 已经是组件定义
                comp_key = f"{prefix}/{key}" if prefix else key
                flat[comp_key] = value
        return flat

    def get_sensor_types(self):
        return self.flat_components


# ------------------ 左侧组件库 ------------------
class ComponentListPanel(QWidget):
    def __init__(self, component_manager, canvas, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.component_manager = component_manager
        self.canvas = canvas

        layout = QVBoxLayout(self)
        layout.setContentsMargins(5,5,5,5)
        layout.setSpacing(10)
        self.setMaximumWidth(250)

        components = self.component_manager.get_sensor_types()
        for comp_type, _ in components.items():
            item_widget = QFrame(self)
            item_layout = QHBoxLayout(item_widget)
            name_label = QLabel(comp_type, item_widget)
            item_layout.addWidget(name_label)

            add_button = QPushButton("+", item_widget)
            add_button.setFixedSize(30,30)
            add_button.clicked.connect(lambda _, t=comp_type: self.canvas.add_component(t))
            item_layout.addWidget(add_button)

            layout.addWidget(item_widget)
        layout.addStretch()


# ------------------ 中间画布上单个组件 ------------------
class ComponentItemWidget(QWidget):
    def __init__(self, comp_type, on_delete, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(5,2,5,2)
        layout.setSpacing(5)

        self.label = QLabel(comp_type, self)
        layout.addWidget(self.label)

        self.delete_button = QPushButton("删除", self)
        self.delete_button.setFixedSize(50,25)
        self.delete_button.clicked.connect(on_delete)
        layout.addWidget(self.delete_button)


# ------------------ 属性面板 ------------------
class PropertyPanel(QWidget):
    def __init__(self, component_manager, canvas, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.component_manager = component_manager
        self.canvas = canvas
        self.setMaximumWidth(350)
        self.setMinimumWidth(300)

        self.main_layout = QVBoxLayout(self)
        self.title_label = QLabel("传感器属性", self)
        self.main_layout.addWidget(self.title_label)

        self.form_layout = QFormLayout()
        self.main_layout.addLayout(self.form_layout)

        self.info_label = QLabel("请点击画布中的组件查看属性", self)
        self.info_label.setWordWrap(True)
        self.main_layout.addWidget(self.info_label)
        self.main_layout.addStretch()

        self.filename_edit = QLineEdit("robot_config")
        self.main_layout.addWidget(QLabel("机器人名称"))
        self.main_layout.addWidget(self.filename_edit)
        # use_videos 和 保存按钮
        self.use_videos_checkbox = QCheckBox("use_videos")
        self.main_layout.addWidget(self.use_videos_checkbox)
        self.save_button = QPushButton("保存配置")
        self.save_button.clicked.connect(self.save_config)
        self.main_layout.addWidget(self.save_button)

        self.apply_button = QPushButton("应用配置")
        self.apply_button.clicked.connect(self.apply_config)
        self.main_layout.addWidget(self.apply_button)

        self.current_sensor_data = None

    def apply_config(self):
        """
        运行生成文件代码
        """
        py_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../utils/j2p.py")
        if not os.path.isfile(py_path):
            print(f"文件不存在: {py_path}")
            return

        try:
            subprocess.run([sys.executable, py_path], check=True)
            print("已成功运行 j2p.py")
        except subprocess.CalledProcessError as e:
            print(f"运行 j2p.py 失败: {e}")


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
        self.title_label.setText(f"{sensor_data['type']} 属性")

        params = sensor_data["params"]

        # id 参数可编辑（生成一次）
        if "id" not in sensor_data:
            sensor_data["id"] = str(uuid.uuid4())
        id_field = QLineEdit(sensor_data["id"])
        id_field.textChanged.connect(lambda val: self.update_id(val))
        self.form_layout.addRow("id", id_field)

        for param_name, param_value in params.items():
            if param_name == "id":
                continue

            # output 多选
            if param_name == "output" and isinstance(param_value, list):
                all_options = self.component_manager.get_sensor_types()[sensor_data["type"]]["default_params"].get("output", [])
                output_layout = QVBoxLayout()
                output_layout.setContentsMargins(0,0,0,0)
                for option in all_options:
                    cb = QCheckBox(option)
                    cb.setChecked(option in param_value)

                    def on_state_changed(state, opt=option):
                        if state == Qt.Checked:
                            if opt not in sensor_data["params"]["output"]:
                                sensor_data["params"]["output"].append(opt)
                        else:
                            if opt in sensor_data["params"]["output"]:
                                sensor_data["params"]["output"].remove(opt)
                    cb.stateChanged.connect(on_state_changed)
                    output_layout.addWidget(cb)

                container = QWidget()
                container.setLayout(output_layout)
                self.form_layout.addRow(param_name, container)
                continue

            # 其它参数
            if isinstance(param_value, int):
                widget = QSpinBox()
                widget.setRange(-999999,999999)
                widget.setValue(param_value)
                widget.valueChanged.connect(lambda val, k=param_name: self.update_param(k,val))
            elif isinstance(param_value,float):
                widget = QDoubleSpinBox()
                widget.setDecimals(4)
                widget.setRange(-999999.0,999999.0)
                widget.setValue(param_value)
                widget.valueChanged.connect(lambda val, k=param_name: self.update_param(k,val))
            elif isinstance(param_value,str):
                if len(param_value)>50:
                    widget = QTextEdit()
                    widget.setPlainText(param_value)
                    widget.textChanged.connect(lambda k=param_name, w=widget: self.update_param(k,w.toPlainText()))
                else:
                    widget = QLineEdit()
                    widget.setText(param_value)
                    widget.textChanged.connect(lambda val, k=param_name: self.update_param(k,val))
            elif isinstance(param_value,list):
                widget = QTextEdit()
                widget.setPlainText("\n".join(map(str,param_value)))
                widget.textChanged.connect(lambda k=param_name, w=widget: self.update_param(
                    k, [line.strip() for line in w.toPlainText().splitlines() if line.strip()]
                ))
            else:
                widget = QLineEdit()
                widget.setText(str(param_value))
                widget.textChanged.connect(lambda val, k=param_name: self.update_param(k,val))

            self.form_layout.addRow(param_name, widget)

    def update_param(self, key, value):
        if self.current_sensor_data:
            self.current_sensor_data["params"][key] = value

    def update_id(self, value):
        if self.current_sensor_data:
            self.current_sensor_data["id"] = value

    def save_config(self):
        config = {
            "use_videos": self.use_videos_checkbox.isChecked(),
            "components": []
        }

        for item in self.canvas.items_map:
            comp_type = item["data"]["type"]
            params = item["data"]["params"]
            comp_id = item["data"].get("id", str(uuid.uuid4()))

            # 获取 outputs_info
            outputs_info = self.component_manager.get_sensor_types()[comp_type].get("outputs_info", {})
            selected_outputs = params.get("output", [])
            full_outputs = {out: outputs_info[out] for out in selected_outputs if out in outputs_info}

            data_to_save = {
                "type": comp_type,
                "params": params,
                "id": comp_id,
                "outputs_info": full_outputs
            }
            config["components"].append(data_to_save)

        save_dir = "../configs"
        os.makedirs(save_dir, exist_ok=True)

        filename = self.filename_edit.text().strip()
        if not filename:
            filename = "robot_config"
        save_path = os.path.join(save_dir, f"{filename}.json")

        with open(save_path, "w", encoding="utf-8") as f:
            json.dump(config, f, indent=4, ensure_ascii=False)
        print(f"配置已保存到 {save_path}")


# ------------------ 中间画布 ------------------
class RobotCanvas(QGraphicsView):
    def __init__(self, property_panel, component_manager, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        self.property_panel = property_panel
        self.component_manager = component_manager
        self.items_map = []

        # 排列参数
        self.item_size = (200,40)
        self.columns = 3
        self.margin = 20

    def add_component(self, comp_type):
        comp_info = self.component_manager.get_sensor_types()[comp_type]
        default_params = comp_info.get("default_params",{})

        def on_delete():
            self._scene.removeItem(proxy)
            self.items_map = [it for it in self.items_map if it["proxy"]!=proxy]
            self.property_panel.clear_properties()
            self.relayout_items()

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
        self.items_map.append({"proxy":proxy,"data":data})
        self._scene.addItem(proxy)
        self.relayout_items()

    def relayout_items(self):
        for idx,item in enumerate(self.items_map):
            row = idx//self.columns
            col = idx%self.columns
            x = col*(self.item_size[0]+self.margin)
            y = row*(self.item_size[1]+self.margin)
            item["proxy"].setPos(x,y)

    def mousePressEvent(self,event):
        super().mousePressEvent(event)
        item = self.itemAt(event.pos())
        if isinstance(item,QGraphicsProxyWidget) and hasattr(item,"sensor_data"):
            self.property_panel.show_sensor_properties(item.sensor_data)


# ------------------ 主窗口 ------------------
class RobotConfigWindow(QWidget):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.resize(1300,800)
        self.setWindowTitle("机器人传感器配置工具")

        #组件管理
        self.component_manager = ComponentManager(
            "../components.yaml"
        )

        layout = QHBoxLayout(self)
        layout.setContentsMargins(5,5,5,5)
        #属性面板
        self.property_panel = PropertyPanel(self.component_manager,None,self)
        #画布
        self.canvas = RobotCanvas(self.property_panel,self.component_manager,self)
        self.property_panel.canvas = self.canvas
        #组件库
        self.component_list = ComponentListPanel(self.component_manager,self.canvas,self)

        layout.addWidget(self.component_list)
        layout.addWidget(self.canvas)
        layout.addWidget(self.property_panel)



