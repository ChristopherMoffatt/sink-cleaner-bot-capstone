# Sink Cleaner Bot (Capstone 2026)

Autonomous hospital sink drain cleaning on a Clearpath Ridgeback with a KUKA arm (ROS Noetic). The robot navigates a hospital room, locates the sink using AprilTags, aligns itself with the sink, and uses the arm to deploy a continuum robot into the drain to clean it.

## Where things live

| Location | What |
| --- | --- |
| `~/catkin_ws/src/Capstone_2026` | This project (sink drain cleaning) and the root of the GitHub repo [sink-cleaner-bot-capstone](https://github.com/ChristopherMoffatt/sink-cleaner-bot-capstone). `~/Capstone_2026` is a link to it, so old paths still work. |
| `~/catkin_ws/src/robot-kuka-ridgeback` | Last year's screw project (`screwed_*` packages). Left in place because its built files point to this path. |
| `apriltag_tracking/` | AprilTag camera pipeline, run as the `apriltag_tracking` Docker container (`./run_ros.sh`). `~/apriltag_tracking` is a link to it. Its `CATKIN_IGNORE` keeps the robot's own catkin build from picking it up; it is built inside Docker. |
| `~/screwed_ws`, `~/catkin_ws_julia_new` | Older workspaces, not used by this project. |

## Hardware

The `.stl` files at the top of the repo are part models: `continuum_disk.stl`, `motor_deck.stl`, `standoff.stl` and `wrist_adapter.stl`.

## Packages

| Package | Contents |
| --- | --- |
| `drain_safety` | PS4 software e-stop (`estop.launch`). |
| `drain_tag_nav` | Spin to find an AprilTag, then drive toward it (`tag_approach.launch`). |
| `drain_2dnav` | Map-based navigation (SLAM, `move_base`). Hidden from ROS by `CATKIN_IGNORE` until `move_base` is installed. |

These two packages are pure Python, so they run straight from `src` without `catkin_make`. If a new package does not show up, run `rospack profile`.

## Copied files

These files are copies from an earlier project, and the originals were left in place. `drain_2dnav` was renamed from `screwed_2dnav`, so the "Copied from" column shows the old name.

| Path | Contents | Copied from |
| --- | --- | --- |
| `drain_2dnav/launch/slam_and_nav.launch` | Live SLAM on `/front/scan`, plus `move_base` | `~/catkin_ws/src/robot-kuka-ridgeback/screwed_stack/screwed_2dnav` |
| `drain_2dnav/launch/slam_online_sync.launch` | SLAM variant set up for simulated time (bag replay) | same |
| `drain_2dnav/launch/move_base.launch` | Navigation on a saved map (`amcl`, `map_server`, `move_base`) | same |
| `drain_2dnav/config/` | Costmap and local planner parameters | same |
| `drain_2dnav/maps/room_map_2026-11-10.pgm`, `.yaml` | Current map, built 2026-11-10 (22.0 × 22.4 m, 5 cm per pixel) | Created for this project |
| `drain_2dnav/maps/map.pgm`, `map.yaml` | Old map from the earlier project, saved August 2025 | same as the launch files |
| `visualization/foxglove.launch` | Foxglove bridge on port 8765 | `~/catkin_ws/src/robot-kuka-ridgeback/screwed_stack/screwed_machine/launch` |
| `reference/robot_config/accessories.launch` | How the LiDAR drivers are started at boot | `/etc/ros/noetic/ros.d` |
| `reference/robot_config/base.launch` | Base robot bringup | `/etc/ros/noetic/ros.d` |
| `reference/robot_config/setup.bash` | Robot ROS environment | `/etc/ros` |
| `reference/robot_config/que36_bringup_setup.bash` | LiDAR models and IP addresses | `~/catkin_ws/src/que36_ridgeback/que36_bringup/config/setup.bash` |

## Notes

- This folder is inside `~/catkin_ws/src`, which is on `ROS_PACKAGE_PATH`.
- `drain_2dnav/CATKIN_IGNORE` was copied as is. It hides the package from catkin and ROS, so `$(find drain_2dnav)` does not resolve yet.
- A 2-minute recording for offline SLAM testing is kept outside this folder at `~/2025-05-09-16-01-47.bag`.
- Files in `reference/` are for reading only. Editing them does not change the robot.
- Installed on the robot: `slam_toolbox`, `map_server`, `foxglove_bridge`.
- Not yet installed on the robot: `move_base`, `amcl`. The launch files in `drain_2dnav/launch/` need `move_base`, so they do not run as they are.

## Driving to an AprilTag

The robot spins in place until it sees tag 0, then drives toward it and stops when the tag is 0.6 m in front of the camera. It drives at 0.15 m/s at most.

### Safety

- **E-stop (circle):** locks all base motion, including the PS4 sticks, stops the Ewellix lift, and shuts down the tag approach.
- **Release (Share + Options):** hold both for 1 second. Driving by joystick works again; autonomy does not restart until you relaunch it.
- **Controller dropout:** if the PS4 controller stops reporting for 1 second, the e-stop latches by itself.
- **Override (hold L1):** the joystick takes over while L1 is held. When you let go, the tag approach continues.
- **LiDAR guard:** the robot does not spin if anything is within about 0.73 m of its centre, and does not drive forward if anything is within 0.25 m of the front bumper. It holds still and logs why.
- **Search limit:** if it has not found the tag after about 1.1 turns, it stops.
- **Physical e-stop:** the software e-stop does not replace it. Keep a hand near it for the first runs.

### Running it

The robot must be off the charger. The `apriltag_tracking` container starts on its own; check that `/tag_detections_json` is publishing when the tag is in view. Run each step in its own SSH terminal.

1. Start the e-stop and leave it running. The tag approach will not move without it.

   ```
   roslaunch drain_safety estop.launch
   ```

2. Optional dry run: the robot does not move, and you can watch what it would do.

   ```
   roslaunch drain_tag_nav tag_approach.launch cmd_vel_topic:=/tag_approach/cmd_vel_test
   rostopic echo /tag_approach/cmd_vel_test
   ```

3. Run it for real:

   ```
   roslaunch drain_tag_nav tag_approach.launch
   ```

   To change the target, add `target_id:=3 stop_distance:=0.8`. Other settings are in `drain_tag_nav/config/tag_approach.yaml`.

4. Watch progress with `rostopic echo /tag_approach/state` (`SEARCH`, `CONFIRM`, `APPROACH`, `DONE` or `FAILED`), and the camera view at `http://192.168.131.1:8080/stream?topic=/tag_detections_image`.

If the e-stop was left latched with no e-stop node running, release it by hand: `rostopic pub -1 /e_stop std_msgs/Bool "data: false"`.

### Known limits

- **Distance accuracy:** the camera is not calibrated (its focal length is a placeholder), so distances can be off by roughly 10–20%. Calibrate it before relying on `stop_distance`.
- **Tag size:** the printed tag 0 is 8 in (0.2032 m) across the black square, but the detector in `~/apriltag_tracking` assumes 10 cm (`tag_size` in `docker/src/ros1_apriltag/launch/apriltag_pipeline.launch`). `tag_approach_node.py` corrects for this by scaling distances by 2.03, using `tag_size_m` and `detector_tag_size_m` in `drain_tag_nav/config/tag_approach.yaml`. Distances shown by the detector itself (`/tag_detections_json`, the video overlay) are still about half the true value. If the detector's `tag_size` is changed to 0.2032, set `detector_tag_size_m` to 0.2032 too, or the correction is applied twice. If you print a tag at a different size, update `tag_size_m`.
- **No obstacle avoidance:** the robot stops at obstacles but does not go around them.
- **Camera transform:** `drain_tag_nav/launch/camera_tf.launch` publishes `base_link` → `camera_optical_frame` (1.5 ft forward, 2 ft up, centred, not tilted). `tag_approach.launch` includes it; run it on its own (`roslaunch drain_tag_nav camera_tf.launch`) to see tag poses in Foxglove. If the camera is moved, update the numbers there. Steering still assumes the camera faces forward on the centreline.

## Viewing the saved map

The robot has no monitor, so the map is viewed from a laptop. This does not move the robot or change the map.

You need the robot powered on, a laptop on either of the robot's networks, and the Foxglove desktop app installed on the laptop. The robot can be reached at two addresses:

- `192.168.131.1` on the robot's own network
- `192.168.1.174` on the shared Wi-Fi network (192.168.1.x)

Use whichever one your laptop is on. The commands below use `192.168.131.1`.

1. Open two terminals on the laptop and run `ssh administrator@192.168.131.1` in each.

2. In the first terminal, publish the saved map:

   ```
   cd ~/Capstone_2026/drain_2dnav/maps
   rosrun map_server map_server room_map_2026-11-10.yaml
   ```

3. In the second terminal, start the Foxglove bridge:

   ```
   roslaunch --screen foxglove_bridge foxglove_bridge.launch port:=8765 address:=0.0.0.0
   ```

4. In Foxglove, choose **Open connection**, then **Foxglove WebSocket**, and enter `ws://192.168.131.1:8765`. On the Wi-Fi network, use `ws://192.168.1.174:8765`.

5. Add a **3D** panel, set the display frame to `map`, and turn on the `/map` topic.

6. When you are done, press Ctrl+C in both terminals.

If Foxglove cannot connect, run `sudo ufw allow 8765/tcp` on the robot and try again.

Do not run `map_server` while SLAM is running, because both publish `/map`.

## Building a new map

The LiDAR drivers start automatically at boot. Stop `map_server` first if it is running. Run SLAM and the bridge each in their own SSH terminal, and press Ctrl+C in each when you are done.

Start SLAM on the front LiDAR:

```
rosparam load $(rospack find slam_toolbox)/config/mapper_params_online_sync.yaml /slam_toolbox
rosrun slam_toolbox sync_slam_toolbox_node __name:=slam_toolbox _base_frame:=base_link _odom_frame:=odom _map_frame:=map _scan_topic:=/front/scan _mode:=mapping _resolution:=0.05 _map_update_interval:=1.0 _minimum_travel_distance:=0.2 _minimum_travel_heading:=0.2 _max_laser_range:=20.0
```

Start the Foxglove bridge:

```
roslaunch --screen foxglove_bridge foxglove_bridge.launch port:=8765 address:=0.0.0.0
```

In the Foxglove desktop app, open a Foxglove WebSocket connection to `ws://192.168.131.1:8765`, or `ws://192.168.1.174:8765` from the Wi-Fi network. Add a 3D panel, set the display frame to `map`, and turn on `/map` and `/front/scan`. Drive the robot slowly around the room with the joystick.

Save the map before stopping SLAM, using a new name so that existing maps are kept:

```
cd ~/Capstone_2026/drain_2dnav/maps
rosrun map_server map_saver -f room_map_YYYY-MM-DD
```
