import rclpy
from rclpy.node import Node

import cv2

from std_msgs.msg import UInt32
from sensor_msgs.msg import CompressedImage

from px4_msgs.msg import VehicleOdometry

from .mission_manager import (
  MissionManager,
  NodeName,
  NodeState,
)


class Vision(Node):
  def __init__(self):
    super().__init__('vision')
    
    self.status_publisher = self.create_publisher(UInt32, "/nodes/vision/status", 10)
    self.image_publisher = self.create_publisher(CompressedImage, "/vision/compressed", 10)
    
    self.command_sub = self.create_subscription(
      UInt32, "/mission/command", self.command_callback, 10
    )
    
    #local debug stream
    #self._cap = self.open_camera()
    #FPS = 30.0

    self.mm = MissionManager()
    self.self_state = NodeState.IDLE
    
    self.timer = self.create_timer(1.0/FPS, self.callback_stream)
      


  def command_callback(self, msg):
    cmd = msg.data
    if self.mm.get_node(cmd) != NodeName.VISION:
      return

    command = self.mm.get_command(cmd)
    if command != self.self_state:
        self.self_state = command
        self.get_logger().info(
            f"State changed to {command.name}"
        )


  def report_status(self):
    msg = UInt32()

    msg.data = self.mm.pack(
        NodeName.VISION,
        self.self_state
    )

    self.status_publisher.publish(msg)


  def callback_stream(self):
    self.report_status()

    if self.self_state != NodeState.BUSY:
      return
      
    if self._cap is None:
      self.self_state = NodeState.ERROR
      self.report_status()
      return
  
    ret, frame = self._cap.read()
    if not ret:
      self.self_state = NodeState.ERROR
      self.get_logger().error("Frame grab failed.")
      return

    msg = CompressedImage()
    msg.format = "jpeg"
    success, encoded = cv2.imencode(".jpg", frame)

    if success:
      msg.data = encoded.tobytes()
      self.image_publisher.publish(msg)

    cv2.imshow("Drone View", frame)
    cv2.waitKey(1)


  def stop(self):
    if self._cap:
      self._cap.release()
    cv2.destroyAllWindows()


  def open_camera(self):
    pipeline = (
      "udpsrc port=5600 ! "
      "application/x-rtp, encoding-name=H264 ! "
      "rtph264depay ! h264parse ! avdec_h264 ! "
      "videoconvert ! "
      "videoscale ! video/x-raw, width=640, height=480 ! "
      "appsink"
    )

    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
      self.get_logger().error("Could not open GStreamer pipeline")
      return None
    return cap


def main(args=None):
  rclpy.init(args=args)

  vision = Vision()
  rclpy.spin(vision)

  vision.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()

