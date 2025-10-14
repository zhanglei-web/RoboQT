import os
import numpy as np
import cv2

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

    # === 生成手臂结构 ===
    arms = components.get("arms", {})
    total_dof = sum(a["dof"] for a in arms.values())
    observations = np.zeros((n_frames, total_dof), dtype=np.float32)
    actions = np.zeros((n_frames, total_dof), dtype=np.float32)
    if create_files:
        np.save(os.path.join(data_dir, "observations.npy"), observations)
        np.save(os.path.join(data_dir, "actions.npy"), actions)

    # === 打印结构树 ===
    print_episode_structure(ep_dir, cameras, arms, total_dof)

    return ep_dir


def print_episode_structure(ep_dir, cameras, arms, total_dof):
    print(f"\n{os.path.basename(ep_dir)}/")
    print("├── images/")
    for cam_name, outputs in cameras.items():
        for out in outputs:
            folder = f"{out}_{cam_name}"
            print(f"│   ├── {folder}/")
            print(f"│   │   ├── frame_000001.jpg")
            print(f"│   │   ├── frame_000002.jpg")
            print(f"│   │   └── ...")

    print("└── data/")
    print(f"    ├── observations.npy   # shape=(N, {total_dof})")
    for arm, info in arms.items():
        for i in range(info["dof"]):
            print(f"    │   ├── {arm}_joint_{i}")
    print(f"    └── actions.npy        # shape=(N, {total_dof})")
    for arm, info in arms.items():
        for i in range(info["dof"]):
            print(f"        ├── {arm}_joint_{i}")


if __name__ == "__main__":
    # 示例：直接运行测试
    components = {
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
    generate_episode_structure(components, create_files=False)
