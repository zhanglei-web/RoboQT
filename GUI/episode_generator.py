import os
import json
import numpy as np
import cv2
os.chdir(os.path.dirname(os.path.abspath(__file__)))

def generate_episode_structure(components, base_dir="./episodes", episode_id=1, n_frames=5, create_files=True):
    """
    根据组件配置生成符合 lerobot 格式的 episode 数据结构。
    
    Args:
        components: dict, e.g.
            {
                "cameras": {
                    "top": ["image", "image_depth"],
                    "left": ["image"],
                    "right": ["image"]
                },
                "arms": {
                    "arm_left": {"dof": 13, "outputs": ["joint_positions"]},
                    "arm_right": {"dof": 13, "outputs": ["joint_positions"]}
                }
            }
        base_dir: 存放所有 episode 的目录
        episode_id: 当前 episode 编号
        n_frames: 每个 episode 的帧数
        create_files: 是否实际创建文件（False 时仅打印结构）
    """
    ep_dir = os.path.join(base_dir, f"episode_{episode_id:06d}")
    images_dir = os.path.join(ep_dir, "images")
    data_dir = os.path.join(ep_dir, "data")

    os.makedirs(images_dir, exist_ok=True)
    os.makedirs(data_dir, exist_ok=True)

    # === 生成相机结构 ===
    cameras = components.get("cameras", {})
    for cam_name, outputs in cameras.items():
        for out in outputs:
            folder = os.path.join(images_dir, f"{out}_{cam_name}")
            os.makedirs(folder, exist_ok=True)
            if create_files:
                for i in range(1, n_frames + 1):
                    img = np.full((240, 320, 3), 120, np.uint8)
                    text = f"{out}_{cam_name} frame {i}"
                    cv2.putText(img, text, (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
                    ext = ".png" if "depth" in out else ".jpg"
                    fname = os.path.join(folder, f"frame_{i:06d}{ext}")
                    cv2.imwrite(fname, img)

 
    arms = components.get("arms", {})
    total_dof = sum(a["dof"] for a in arms.values())
    observations = np.zeros((n_frames, total_dof), dtype=np.float32)
    actions = np.zeros((n_frames, total_dof), dtype=np.float32)
    if create_files:
        np.save(os.path.join(data_dir, "observations.npy"), observations)
        np.save(os.path.join(data_dir, "actions.npy"), actions)


    print_episode_structure(ep_dir, cameras, arms, total_dof)

    return ep_dir

def parse_robot_config_to_episode_components(json_path):
    with open(json_path, "r", encoding="utf-8") as f:
        config = json.load(f)

    components = {"cameras": {}, "arms": {}}

    for idx, comp in enumerate(config.get("components", [])):
        comp_type = comp["type"]
        params = comp.get("params", {})
        outputs = params.get("output", [])
        comp_id = comp.get("id", f"comp_{idx}")


        if comp_type.startswith("camera/"):
            for out in outputs:
                cam_name = f"{comp_id}" 
                components["cameras"][cam_name] = [out]


        elif comp_type.startswith("arm/") or comp_type.startswith("robot_arm"):
            arm_name = f"robot_arm_{comp_id}" 
            outputs_info = comp.get("outputs_info", {}).get("pose", {})
            motors = outputs_info.get("motors", {})
            dof = len(motors) if motors else 6

            components["arms"][arm_name] = {
                "dof": dof,
                "outputs": outputs
            }

    return components


def print_episode_structure(ep_dir, cameras, arms, total_dof):
    print(f"\n{os.path.basename(ep_dir)}/")
    print("├── images/")
    for cam_name, outputs in cameras.items():
        for out in outputs:
            folder = f"{cam_name}"
            print(f"│   ├── {folder}/")
            print(f"│   │   ├── frame_000001.jpg")
            print(f"│   │   ├── frame_000002.jpg")
            print(f"│   │   └── ...")
    print("│")
    print("└── data/")
    print(f"    ├── observations   # shape=(1, {total_dof})")
    for arm, info in arms.items():
        print(f"    │   ├── {arm}/")
        for i in range(min(3, info['dof'])):
            print(f"    │   │   ├── joint_{i}")
        if info['dof'] > 3:
            print(f"    │   │   └── ...")
    print(f"    └── actions        # shape=(1, {total_dof})")
    for arm, info in arms.items():
        print(f"        ├── {arm}/")
        for i in range(min(3, info['dof'])):
            print(f"        │   ├── joint_{i}")
        if info['dof'] > 3:
            print(f"        │   └── ...")



if __name__ == "__main__":
    json_path = "../config/robot_config.json"
    components = parse_robot_config_to_episode_components(json_path)
    print("解析结果：")
    print(json.dumps(components, indent=4))
    generate_episode_structure(components, create_files=False)
