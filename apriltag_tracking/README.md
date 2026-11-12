# apriltag_tracking

Real-time AprilTag detection on a USB camera, running ROS 1 Noetic in Docker on Linux.

The `apriltag_tracking` container opens the first external (USB) camera it finds, detects
AprilTags, publishes poses/TF on ROS topics, and serves the annotated video to a web browser.
Everything starts with the container — there is nothing to run inside it.

## Run

```bash
./run_ros.sh
```

This builds the image and starts the container. The container uses `restart: unless-stopped`,
so it also comes back (and starts streaming) on its own after a reboot.

View the stream in a browser:

```text
http://<host-ip>:8080/stream?topic=/tag_detections_image
```

Other useful commands:

```bash
docker logs -f apriltag_tracking                                    # see which camera was picked
rostopic echo /tag_poses                                            # on the host, topics are on its ROS master
docker compose -f docker/docker-compose.yml down                    # stop
```

## Camera selection

The camera node scans `/dev/video*` in order and uses the first USB camera that delivers frames.
If none is connected (or it is unplugged), it keeps rescanning until one appears.

To pin a specific device instead:

```bash
CAMERA_DEVICE=/dev/video2 ./run_ros.sh
```

Resolution, frame rate, tag family and tag size are set in
`docker/src/ros1_apriltag/launch/apriltag_pipeline.launch` (re-run `./run_ros.sh` after editing).

## Notes

* The container uses host networking and joins the ROS master already running on the host
  (`http://localhost:11311`), so its topics appear in `rostopic list` on the host. It does not
  start a `roscore` of its own: it waits (`roslaunch --wait`) until the host's master is up.
  If the host's master is restarted, restart the container too: `docker restart apriltag_tracking`.
* Ports 8080 (video) and 9090 (rosbridge websocket) are served directly on the host.
* Camera intrinsics are approximated (focal length = 0.8 × image width), so distances are estimates.

## Printable tags

Family `tag36h11`, IDs 0–5 are in `tags/`. Regenerate with `python3 generate_tag.py`
(needs `opencv-python` and `numpy`).

## ROS topics

| Topic | Message Type | Description |
|---|---|---|
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | Camera feed |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Approximate intrinsics |
| `/tag_detections_image` | `sensor_msgs/Image` | Annotated image with 3D axes and bounding boxes |
| `/tag_poses` | `geometry_msgs/PoseArray` | 3D poses (position and quaternion orientation) |
| `/tag_detections_json` | `std_msgs/String` | JSON string containing all detected tags |
| `/tf` | `tf/tfMessage` | Transforms from `camera_optical_frame` to `tag_<id>` |
