
# nav2_pose_saver

A modular ROS 2 node for persisting and restoring the robot's last known pose using the `/amcl_pose` topic. This is useful in applications where the robot is never manually moved (e.g. warehouse robots), and it enables automatic re-localization.

## Features
-  Save the current AMCL pose to disk periodically
-  Write safely using an atomic temp-file swap
-  Restore the robot pose automatically on startup (optional)
-  Restore pose via service when AMCL restarts
-  Start/Stop pose saving dynamically with services
-  Reset pose file to zeros (e.g. to clear old pose data)
-  Parameter configurable save interval and file path
-  Supports saving to multiple disk locations

## Installation

```bash
cd ~/pose_ws/src
git clone https://github.com/<your-username>/nav2_pose_saver.git
cd ~/pose_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select nav2_pose_saver --symlink-install
source install/setup.bash
```

## Usage

Launch with default parameters:

```bash
ros2 launch nav2_pose_saver pose_saver.launch.py
```

Or override parameters on launch:

```bash
ros2 launch nav2_pose_saver pose_saver.launch.py \
  auto_start_saving:=true \
  auto_restore_pose:=true \
  save_interval_sec:=3.0 \
  pose_file_path:=/tmp/my_pose.yaml
```

✅ **Note**: If using simulation, pass `use_sim_time:=true` to both this node and your navigation stack.

---

## Parameters

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `save_interval_sec` | double | `5.0` | Time interval in seconds between pose saves |
| `pose_file_path` | string | `~/.ros/last_known_pose.yaml` | Path used for reading/writing saved pose |
| `auto_start_saving` | bool | `true` | Start saving immediately on launch |
| `auto_restore_pose` | bool | `false` | Publish pose to `/initialpose` on startup |

---

## Services

| Service Name | Description |
|--------------|-------------|
| `/start_pose_saver` | Starts periodic pose saving |
| `/stop_pose_saver` | Stops periodic pose saving |
| `/localise_at_last_known_position` | Publishes last saved pose to `/initialpose` |
| `/reset_last_known_pose` | Resets the saved pose to zeros |

Call a service like this:

```bash
ros2 service call /start_pose_saver std_srvs/srv/Trigger
```

---

## File Save Locations

When active, poses are saved to:
- `/tmp/pose_saver.yaml`
- `<install_space>/share/nav2_pose_saver/config/last_known_pose.yaml`

The main pose file used for restoration is:
- As defined in the `pose_file_path` parameter (`~/.ros/last_known_pose.yaml` by default)

All writes are performed atomically using `.tmp` + `std::rename()` to prevent corruption.

---
  Absolutely! Here's the updated **"Example Workflow"** section for the `README.md`, now clearly explaining the correct **startup order** and how to **relocalize properly** using `auto_restore_pose`.

---

## Example Workflow

When re-localizing the robot using a previously saved pose:

1. **First**, launch the **pose saver node** with `auto_restore_pose` set to `true`:

   ```bash
   ros2 launch nav2_pose_saver pose_saver.launch.py auto_restore_pose:=true use_sim_time:=true
   ```

2. **Then**, launch your **navigation stack** (e.g. using Nav2 or TurtleBot3):

   ```bash
   ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true
   ```

> ✅ **Important:**  
> Pose saver must be started **before** the navigation stack to ensure that AMCL receives the `/initialpose` message in time for proper localization.

If you start navigation first, AMCL may already have initialized itself and **ignore the `/initialpose`** message sent afterward.

To retry localization manually:
```bash
ros2 service call /localise_at_last_known_position std_srvs/srv/Trigger
```

## License

This project is licensed under the [Apache License 2.0](LICENSE).

---

## Contributing

Contributions are welcome. If you'd like to improve this package, add new features, or fix bugs:

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/my-feature`)
3. Commit your changes (`git commit -am 'Add new feature'`)
4. Push to the branch (`git push origin feature/my-feature`)
5. Open a Pull Request

Please format code using ROS 2 C++ guidelines and keep commits clean.

---

## Compatibility

- ✅ Tested on ROS 2 Jazzy with Turtlebot3
- Should work with any AMCL-compatible setup using `/amcl_pose` and `/initialpose`

---

> Type !ex to export everything  
> Type !h to see all hotkeys  
> And if this saved you hours, consider supporting me: [☕ BuyMeACoffee](https://www.buymeacoffee.com/zerothlaw)  
