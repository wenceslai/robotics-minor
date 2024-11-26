from gpiozero import PWMOutputDevice, DigitalOutputDevice
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time


class MotorControl(Node):
   def __init__(self):
      super().__init__("motor_control")
      #self._left_client = self.create_client(
      #   SetMotorSpeed,
      #   "/mirte/set_left_speed")
      #self._right_client = self.create_client(
      #   SetMotorSpeed,
      #   "/mirte/set_right_speed")
      self._subscription = self.create_subscription(
         Bool,
         "/is_aligned",
         self.receive_message_callback,
         1
      )
      self.misalignment_frames = 0
      self.alignment_frames = 0
      self.client_futures = []
      
      self.ENA = PWMOutputDevice(18)  # PWM for left motor
      self.ENB = PWMOutputDevice(19)  # PWM for right motor
      self.IN1 = DigitalOutputDevice(23)  # Direction pin for left motor
      self.IN2 = DigitalOutputDevice(24)  # Direction pin for left motor
      self.IN3 = DigitalOutputDevice(25)  # Direction pin for right motor
      self.IN4 = DigitalOutputDevice(26)  # Direction pin for right motor

      self.get_logger().info("Initialized example client")

   def set_motor(self, motor, speed):
      if motor == 'right':
          pwm = self.ENA
          in1, in2 = self.IN1, self.IN2
      elif motor == 'left':
          pwm = self.ENB
          in1, in2 = self.IN3, self.IN4
      else:
          print("Invalid motor specified. Use 'left' or 'right'.")
          return

      # Determine direction
      if speed < 0:
          in1.on()
          in2.off()
      elif speed > 0:
          in1.off()
          in2.on()
      else:
          # Stop the motor
          in1.off()
          in2.off()
          pwm.value = 0
          return

      # Set speed using PWM (0 to 1 range)
      pwm.value = min(abs(speed) / 100, 1.0)


   def set_speeds_async(self, left, right):
      self.set_motor("left", left)
      self.set_motor("right", right)
      #left_request = SetMotorSpeed.Request()
      #right_request = SetMotorSpeed.Request()
      #left_request.speed = left
      #right_request.speed = right
      #self.client_futures.append((
      #   self._left_client.call_async(left_request),
      #   self._right_client.call_async(right_request)
      #))

   def receive_message_callback(self, message):
      is_aligned = message.data
      self.get_logger().info("Is aligned: " + str(is_aligned))

      if self.alignment_frames > 10:
         self.get_logger().info("Setting speed to 20")
         self.set_speeds_async(100, 100)
         return
      else:
         self.set_speeds_async(0, 0)
      if not is_aligned:
         self.alignment_frames = 0
         self.misalignment_frames += 1
         if self.misalignment_frames > 3:
            self.get_logger().info("Setting speed to 10")
            self.set_speeds_async(100, -100)
            time.sleep(0.5)
            self.set_speeds_async(0, 0)
            self.misalignment_frames = 0
      else:
         self.misalignment_frames = 0
         self.alignment_frames += 1
         self.get_logger().info("Setting speed to 0")
         self.set_speeds_async(0, 0)


   def spin(self):
      while rclpy.ok():
         rclpy.spin_once(self)
         incomplete_futures = []
         for (left, right) in self.client_futures:
            if left.done() and right.done():
               res = (left.result().status, right.result().status)
               self.get_logger().info("Result: " + str(res))
            else:
               incomplete_futures.append((left, right))
         self.client_futures = incomplete_futures

def main():
   rclpy.init()
   motor_control = MotorControl()
   try:
       rclpy.spin(motor_control)
   except KeyboardInterrupt:
       motor_control.set_speeds_async(0, 0)
       rclpy.shutdown()
