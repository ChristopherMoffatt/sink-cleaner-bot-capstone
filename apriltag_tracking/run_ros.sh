#!/bin/bash
# Build and start the apriltag_tracking container.
# The container starts the camera stream + AprilTag detection on its own,
# and comes back automatically after a reboot (restart: unless-stopped).
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/docker"

docker compose up -d --build

HOST_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
echo ""
echo "=========================================================="
echo " apriltag_tracking is running"
echo "   Stream: http://${HOST_IP:-localhost}:8080/stream?topic=/tag_detections_image"
echo "   Logs:   docker logs -f apriltag_tracking"
echo "   Poses:  rostopic echo /tag_poses"
echo "   Stop:   docker compose -f docker/docker-compose.yml down"
echo "=========================================================="
