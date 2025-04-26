---

# nav2_pose_saver

A modular and lightweight ROS 2 node for persisting and restoring the robot's last known pose via the `/amcl_pose` topic.  
Designed for autonomous robots operating in controlled environments (e.g., warehouses, industrial sites), this tool enhances system robustness by enabling automatic re-localization after system restarts, crashes, or power interruptions.

---

## Features

- ✅ Periodically saves the robot’s AMCL pose to disk
- ✅ Safe file writing using atomic temp-file swap
- ✅ Automatic pose restoration at startup or AMCL reconnection
- ✅ Dynamic service-based control (start, stop, restore, reset)
- ✅ Flexible parameterization for intervals, file locations, and behavior
- ✅ Multi-location pose persistence (temporary, install-space, and custom)
- ✅ Intelligent monitoring of AMCL connectivity (no need to restart Pose Saver)
- ✅ Fully compatible with both real-world robots and simulation environments

---

## Installation

```bash
cd ~/pose_ws/src
git clone https://github.com/<your-username>/nav2_pose_saver.git
cd ~/pose_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select nav2_pose_saver --symlink-install
source install/setup.bash
```

---

## Usage

```bash
ros2 launch nav2_pose_saver pose_saver.launch.py
```

> Configure runtime behavior using launch parameters as needed.

When `auto_restore_pose` is set to `true`, the node will automatically re-publish the last known pose whenever AMCL reconnects — even after navigation crashes or restarts.

---

## Parameters

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `save_interval_sec` | `double` | `5.0` | Interval in seconds between automatic pose saves |
| `pose_file_path` | `string` | `~/.ros/last_known_pose.yaml` | File path for loading and saving poses |
| `auto_start_saving` | `bool` | `true` | Whether to start saving poses automatically at startup |
| `auto_restore_pose` | `bool` | `false` | Whether to automatically restore the pose on startup and monitor AMCL for reconnection |

---

## Services

| Service Name | Description |
|--------------|-------------|
| `/start_pose_saver` | Start periodic pose saving |
| `/stop_pose_saver` | Stop periodic pose saving |
| `/localise_at_last_known_position` | Publish the last saved pose to `/initialpose` |
| `/reset_last_known_pose` | Reset the stored pose to a default zero pose |

### Example Service Call:

```bash
ros2 service call /start_pose_saver std_srvs/srv/Trigger
```

---

## File Persistence

Pose data is saved to:
- `/tmp/pose_saver.yaml`
- `<install_space>/share/nav2_pose_saver/config/last_known_pose.yaml`
- (Optional) A user-specified path via the `pose_file_path` parameter

All file writes use an atomic save mechanism (temp file + rename) to ensure filesystem consistency and prevent partial writes.

---

## Behavior and Workflow

- Start `nav2_pose_saver` node **before** or **along with** the Navigation stack.
- If `auto_restore_pose` is enabled:
  - Waits for `/clock` synchronization (if simulation time is used).
  - Waits for AMCL to subscribe to `/initialpose`.
  - Publishes the last known pose once AMCL is ready.
  - Continues **monitoring AMCL subscriptions**:
    - If AMCL unsubscribes (e.g., due to a crash) and later re-subscribes, the pose is automatically re-published without requiring any manual intervention or node restarts.

✅ This enables **seamless re-localization** after navigation stack restarts.

---

## License

This project is licensed under the [Apache License 2.0](LICENSE).

---

## Contributing

Contributions are welcome!  
Please follow the ROS 2 C++ style guide when submitting pull requests.

**Steps to contribute:**

1. Fork this repository
2. Create a feature branch (`git checkout -b feature/my-feature`)
3. Commit your changes (`git commit -am 'Add new feature'`)
4. Push your branch (`git push origin feature/my-feature`)
5. Open a Pull Request (PR)

---

## Compatibility

- ✅ Tested on **ROS 2 Jazzy**
- ✅ Compatible with **TurtleBot3**, **simulation**, and **real-world deployments**
- ✅ Should generalize well to any robot using AMCL for localization

---

# 🛡️ Stable. Lightweight. Reliable.

---
