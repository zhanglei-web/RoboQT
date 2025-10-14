import os
import json
import uuid

def generate_config(components, component_manager, use_videos=False, filename="robot_config"):
    """
    根据画布上的组件生成配置文件
    """
    config = {
        "use_videos": use_videos,
        "components": []
    }

    for item in components:
        comp_type = item["data"]["type"]
        params = item["data"]["params"]
        comp_id = item["data"].get("id", str(uuid.uuid4()))

        # 获取 outputs_info
        outputs_info = component_manager.get_flat_components()[comp_type].get("outputs_info", {})
        selected_outputs = params.get("output", [])
        full_outputs = {out: outputs_info[out] for out in selected_outputs if out in outputs_info}

        data_to_save = {
            "type": comp_type,
            "params": params,
            "id": comp_id,
            "outputs_info": full_outputs
        }
        config["components"].append(data_to_save)

    # 保存文件
    save_dir = "/home/ray/WanX-EI-Studio/operating_platform/robot/RoboQT/test/config"
    os.makedirs(save_dir, exist_ok=True)
    save_path = os.path.join(save_dir, f"{filename}.json")

    with open(save_path, "w", encoding="utf-8") as f:
        json.dump(config, f, indent=4, ensure_ascii=False)

    return save_path
