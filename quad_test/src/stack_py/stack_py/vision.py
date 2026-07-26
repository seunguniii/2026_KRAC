import rclpy
from rclpy.node import Node

import cv2

from std_msgs.msg import UInt32
from sensor_msgs.msg import CompressedImage

from .mission_manager import (
  MissionManager,
  NodeName,
  NodeState,
)

#TODO: currently image frames are shared via ROS topics
#      creates unnecessary processes for serialize/desirializing frames
#      if problems such as excessive badnwith occur, 
#      use local ports using gstreamer tee at udp port 5600 etc.
#      ...or merge vision, marker and yolo node into a single node
class Vision(Node):
  def __init__(self):
    super().__init__('vision')
    
    self.status_publisher = self.create_publisher(UInt32, "/nodes/vision/status", 10)
    self.image_publisher = self.create_publisher(CompressedImage, "/nodes/vision/stream", 10)
    
    self.command_subscriber = self.create_subscription(
      UInt32, "/mission/command", self.command_callback, 10
    )
    
    self._cap = self.open_camera()
    FPS = 30.0

    self.mm = MissionManager()
    self.self_state = NodeState.IDLE
    
    self.timer = self.create_timer(1.0/FPS, self.main_callback)
      


  def command_callback(self, msg):
    cmd = msg.data
    if self.mm.get_node(cmd) != NodeName.VISION:
      return

    command = self.mm.get_command(cmd)
    if command != self.self_state:
        self.self_state = command
        self.get_logger().info("Command recieved from MISSION")


  def report_status(self):
    msg = UInt32()

    msg.data = self.mm.pack(
        NodeName.VISION,
        self.self_state
    )

    self.status_publisher.publish(msg)


  #main logic
  def main_callback(self):
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

  def stop(self):
    if self._cap:
      self._cap.release()
    cv2.destroyAllWindows()


  def open_camera(self):
    #TODO: set appropriate pipeline according to environment
    #      should use below for orin
    #      suggested values
    #      fps = 30.0
    #      flip_method = 0 : no flip/rotation
    #                        use appropriate value if video orientation is not aligned
    #pipeline = (
    #  "nvarguscamerasrc sensor-id=0 ! "
    #  "video/x-raw(memory:NVMM), width={width}, height={height},format=NV12,framerate={fps}/1 ! "
    #  "nvvidconv flip-method={flip_method} !"
    #  "video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format BGR ! appsink"
    #)
    pipeline = (
      "udpsrc port=5600 ! "
      "application/x-rtp, encoding-name=H264 ! "
      "rtph264depay ! h264parse ! avdec_h264 ! "
      "videoconvert ! "
      "videoscale ! video/x-raw, width=1280, height=720 ! "
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

