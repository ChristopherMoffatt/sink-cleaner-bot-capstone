#!/usr/bin/env python3
"""
Generate printable AprilTag markers (tag36h11 by default).
Can be printed or displayed on your phone screen to test the detector!
"""

import os
import sys

def main():
    try:
        import cv2
        import numpy as np
    except ImportError:
        print("Please install opencv-python: pip install opencv-python")
        sys.exit(1)

    output_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tags")
    os.makedirs(output_dir, exist_ok=True)

    # Use AprilTag 36h11 dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

    print("Generating sample AprilTags (Family: tag36h11)...")
    tag_ids = [0, 1, 2, 3, 4, 5]
    img_size = 500  # pixels
    border_bits = 1

    for tag_id in tag_ids:
        tag_img = cv2.aruco.generateImageMarker(aruco_dict, tag_id, img_size, borderBits=border_bits)
        # Add a white margin around the tag for better detection
        margin = 50
        tag_with_margin = cv2.copyMakeBorder(
            tag_img,
            margin, margin, margin, margin,
            cv2.BORDER_CONSTANT,
            value=[255, 255, 255]
        )
        
        # Add text label at the bottom
        label = f"AprilTag 36h11 - ID: {tag_id}"
        cv2.putText(
            tag_with_margin,
            label,
            (margin, img_size + margin + 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 0),
            2,
            cv2.LINE_AA
        )
        
        filename = os.path.join(output_dir, f"apriltag_36h11_id_{tag_id}.png")
        cv2.imwrite(filename, tag_with_margin)
        print(f"  [+] Saved: {filename}")

    print(f"\nTags saved to: {output_dir}")
    print("You can open these images on your phone screen or print them to test detection.")

if __name__ == "__main__":
    main()
