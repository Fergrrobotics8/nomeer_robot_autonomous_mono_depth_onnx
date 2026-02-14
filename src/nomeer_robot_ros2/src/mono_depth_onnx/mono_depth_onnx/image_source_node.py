#!/usr/bin/env python3
"""
Image Source Node
Publishes RGB images from different sources:
- Video file
- Image folder
- Webcam (optional)

Supports various formats and frame rates.
"""

import os
import cv2
import glob
from pathlib import Path
from typing import List, Optional
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImageSourceNode(Node):
    """Publishes RGB images from various sources."""

    SOURCE_VIDEO = "video"
    SOURCE_FOLDER = "folder"
    SOURCE_WEBCAM = "webcam"

    def __init__(self):
        super().__init__('image_source')
        
        # Declare parameters
        self.declare_parameter('source_type', self.SOURCE_FOLDER)  # 'video', 'folder', 'webcam'
        self.declare_parameter('source_path', 'data/images')  # path to video or folder
        self.declare_parameter('publish_frequency', 10.0)  # Hz
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('loop_video', True)
        
        # Get parameters
        self.source_type = self.get_parameter('source_type').value
        self.source_path = self.get_parameter('source_path').value
        self.publish_freq = self.get_parameter('publish_frequency').value
        self.img_width = self.get_parameter('image_width').value
        self.img_height = self.get_parameter('image_height').value
        self.loop_video = self.get_parameter('loop_video').value
        
        # Initialize source
        self.cap = None
        self.image_list: List[str] = []
        self.current_image_idx = 0
        self.frame_count = 0
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Publisher
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # Initialize based on source type
        if not self._initialize_source():
            self.get_logger().error('Failed to initialize image source')
            return
        
        # Timer for publishing
        self.timer = self.create_timer(1.0 / self.publish_freq, self.publish_image)
        
        self.get_logger().info(
            f'Image Source Node initialized\n'
            f'  Source type: {self.source_type}\n'
            f'  Source path: {self.source_path}\n'
            f'  Publish frequency: {self.publish_freq}Hz\n'
            f'  Output size: {self.img_width}x{self.img_height}'
        )

    def _initialize_source(self) -> bool:
        """Initialize image source based on type."""
        if self.source_type == self.SOURCE_VIDEO:
            return self._initialize_video()
        elif self.source_type == self.SOURCE_FOLDER:
            return self._initialize_folder()
        elif self.source_type == self.SOURCE_WEBCAM:
            return self._initialize_webcam()
        else:
            self.get_logger().error(f'Unknown source type: {self.source_type}')
            return False

    def _initialize_video(self) -> bool:
        """Initialize video file source."""
        if not os.path.exists(self.source_path):
            self.get_logger().error(f'Video file not found: {self.source_path}')
            return False
        
        self.cap = cv2.VideoCapture(self.source_path)
        
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open video: {self.source_path}')
            return False
        
        frame_count = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        
        self.get_logger().info(
            f'Video source initialized\n'
            f'  Total frames: {frame_count}\n'
            f'  FPS: {fps}'
        )
        return True

    def _initialize_folder(self) -> bool:
        """Initialize image folder source."""
        if not os.path.isdir(self.source_path):
            self.get_logger().error(f'Folder not found: {self.source_path}')
            return False
        
        # Find all image files
        image_extensions = ['*.jpg', '*.jpeg', '*.png', '*.bmp', '*.tiff']
        self.image_list = []
        
        for ext in image_extensions:
            self.image_list.extend(glob.glob(
                os.path.join(self.source_path, ext)
            ))
            self.image_list.extend(glob.glob(
                os.path.join(self.source_path, ext.upper())
            ))
        
        self.image_list = sorted(list(set(self.image_list)))
        
        if not self.image_list:
            self.get_logger().error(f'No images found in: {self.source_path}')
            return False
        
        self.get_logger().info(
            f'Image folder source initialized\n'
            f'  Total images: {len(self.image_list)}\n'
            f'  First image: {os.path.basename(self.image_list[0])}'
        )
        return True

    def _initialize_webcam(self) -> bool:
        """Initialize webcam source."""
        try:
            camera_id = int(self.source_path) if self.source_path.isdigit() else 0
        except:
            camera_id = 0
        
        self.cap = cv2.VideoCapture(camera_id)
        
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open webcam: {camera_id}')
            return False
        
        # Set properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.img_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_height)
        
        self.get_logger().info(f'Webcam source initialized (device: {camera_id})')
        return True

    def publish_image(self):
        """Publish next frame."""
        frame = None
        
        if self.source_type == self.SOURCE_VIDEO:
            frame = self._read_video_frame()
        elif self.source_type == self.SOURCE_FOLDER:
            frame = self._read_folder_frame()
        elif self.source_type == self.SOURCE_WEBCAM:
            frame = self._read_webcam_frame()
        
        if frame is None:
            return
        
        # Resize if needed
        if frame.shape[0] != self.img_height or frame.shape[1] != self.img_width:
            frame = cv2.resize(frame, (self.img_width, self.img_height),
                             interpolation=cv2.INTER_LINEAR)
        
        # Convert to ROS Image message
        try:
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            ros_image.header.frame_id = 'camera'
            ros_image.header.stamp = self.get_clock().now().to_msg()
            self.image_pub.publish(ros_image)
            self.frame_count += 1
            
            if self.frame_count % 30 == 0:
                self.get_logger().debug(f'Published frame {self.frame_count}')
        
        except Exception as e:
            self.get_logger().error(f'Failed to publish image: {e}')

    def _read_video_frame(self) -> Optional[cv2.Mat]:
        """Read frame from video."""
        if self.cap is None:
            return None
        
        ret, frame = self.cap.read()
        
        if not ret:
            if self.loop_video:
                self.get_logger().info('Looping video')
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = self.cap.read()
            else:
                self.get_logger().info('End of video reached')
                return None
        
        return frame

    def _read_folder_frame(self) -> Optional[cv2.Mat]:
        """Read next frame from folder."""
        if self.current_image_idx >= len(self.image_list):
            if self.loop_video:
                self.current_image_idx = 0
            else:
                return None
        
        image_path = self.image_list[self.current_image_idx]
        frame = cv2.imread(image_path)
        
        if frame is None:
            self.get_logger().warn(f'Failed to read image: {image_path}')
            self.current_image_idx += 1
            return None
        
        self.current_image_idx += 1
        return frame

    def _read_webcam_frame(self) -> Optional[cv2.Mat]:
        """Read frame from webcam."""
        if self.cap is None:
            return None
        
        ret, frame = self.cap.read()
        return frame if ret else None

    def destroy_node(self):
        """Clean up resources."""
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImageSourceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
