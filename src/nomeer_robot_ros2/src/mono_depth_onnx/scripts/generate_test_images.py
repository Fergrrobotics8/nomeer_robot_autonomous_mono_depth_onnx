#!/usr/bin/env python3
"""
Generate test images for depth estimation.
Creates several synthetic test images for testing the pipeline.
"""

import cv2
import numpy as np
import os
from pathlib import Path


def create_test_images(output_dir: str = "data/images", num_images: int = 5):
    """Generate synthetic test images."""
    os.makedirs(output_dir, exist_ok=True)
    
    print(f"Generating {num_images} test images to {output_dir}/")
    
    for i in range(num_images):
        # Create image (height, width, channels)
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Background gradient
        for y in range(480):
            img[y, :] = [50 + int(100 * y / 480), 50, 100]
        
        # Add some shapes at different depths
        if i == 0:
            # Image 1: Horizontal lines (simulating distance)
            for j in range(0, 480, 40):
                cv2.line(img, (0, j), (640, j), (0, 255, 0), 2)
        
        elif i == 1:
            # Image 2: Circles
            for j in range(5):
                cx, cy = 320, 240
                r = 50 + j * 30
                cv2.circle(img, (cx, cy), r, (255, 100, 0), 2)
        
        elif i == 2:
            # Image 3: Rectangle in center (simulating obstacle)
            cv2.rectangle(img, (200, 150), (440, 330), (0, 0, 255), -1)
            cv2.putText(img, "OBSTACLE", (250, 250),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        elif i == 3:
            # Image 4: Multiple objects
            cv2.circle(img, (150, 150), 80, (255, 0, 0), -1)
            cv2.circle(img, (490, 150), 80, (0, 255, 0), -1)
            cv2.rectangle(img, (230, 300), (410, 420), (0, 0, 255), -1)
        
        else:
            # Image 5: Text and patterns
            cv2.putText(img, "Test Image", (150, 100),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 0), 2)
            for x in range(0, 640, 50):
                cv2.line(img, (x, 0), (x, 480), (100, 100, 100), 1)
        
        # Save image
        filename = os.path.join(output_dir, f"test_image_{i+1:03d}.jpg")
        cv2.imwrite(filename, img)
        print(f"  ✓ Created {filename}")
    
    print(f"\n✓ Generated {num_images} test images successfully!")
    print(f"Location: {output_dir}/")


if __name__ == '__main__':
    create_test_images()
