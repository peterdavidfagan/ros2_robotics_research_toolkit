#!/usr/bin/env python3
"""Demonstrating policy deployment for a policy that accepts a single image as input."""

import rclpy
import numpy as np
from cv_bridge import CvBridge

from moveit.policies import Policy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import ServoCommandType
from std_srvs.srv import SetBool

from panda_policy_deployment_demos.panda_policy_deployment_demos_parameters import policy as params

#import jax
#import flax
#import flax.linen as nn


#class CNN(nn.Module):
#  """A simple CNN model."""

#  @nn.compact
#  def __call__(self, x):
#    x = nn.Conv(features=32, kernel_size=(3, 3))(x)
#    x = nn.relu(x)
#    x = nn.avg_pool(x, window_shape=(2, 2), strides=(2, 2))
#    x = nn.Conv(features=64, kernel_size=(3, 3))(x)
#    x = nn.relu(x)
#    x = nn.avg_pool(x, window_shape=(2, 2), strides=(2, 2))
#    x = x.reshape((x.shape[0], -1))  # flatten
#    x = nn.Dense(features=256)(x)
#    x = nn.relu(x)
#    x = nn.Dense(features=7)(x)

    # potential to apply constraints to network outputs to 
    # ensure poses are within the robot workspace

#    return x

class SingleImagePolicy(Policy):
    """Dummy policy for testing."""

    def __init__(self):
        super().__init__(params)
        self._logger.info("Dummy policy initialized")
        
        # use CvBridge to convert sensor_msgs/Image to numpy array
        self.cv_bridge = CvBridge()
        
        # Dummy neural network
 #       self.cnn = CNN()
 #       self.cnn_params = self.cnn.init(jax.random.PRNGKey(0), jax.numpy.ones((360, 640, 3)))
        
        # initialize servo service client
        self.servo_command_client = self.create_client(ServoCommandType, "/servo_node/switch_command_type")
        while not self.servo_command_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.servo_pause_client = self.create_client(SetBool, "/servo_node/pause_servo")
        while not self.servo_pause_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")


    def set_servo_command_type(self, command_type):
        """Sets the servo command type."""
        request = ServoCommandType.Request()
        request.command_type = command_type
        self.future = self.servo_command_client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result():
            self.get_logger().info("Servo command type set to: %s" % command_type)
        else:
            self.get_logger().error("Failed to set servo command type: %s" % command_type)


    def forward(self, sensor_msg):
        """Forward pass of the policy."""
        if self._is_active:
            # convert image data to jax array
            cv_img = self.cv_bridge.imgmsg_to_cv2(sensor_msg, desired_encoding="rgb8")
  #          image = jax.numpy.array(cv_img)
  #          self._logger.debug("Image shape: {}".format(image.shape))

            # add code to perform forward pass through your neural network here
  #          dummy_action = self.cnn.apply(self.cnn_params, image)
  #          self._logger.debug("Dummy action shape: {}".format(dummy_action.shape))

            # For this demo, we will just publish random target poses
            target = PoseStamped()
            target.header.stamp = self.get_clock().now().to_msg()
            target.header.frame_id = "panda_link0"
            target.pose.position.x = np.random.uniform(0.3, 0.7)
            target.pose.position.y = np.random.uniform(-0.2, 0.2)
            target.pose.position.z = np.random.uniform(0.25, 0.55)
            target.pose.orientation.w = 0.0
            target.pose.orientation.x = 0.924
            target.pose.orientation.y = -0.382
            target.pose.orientation.z = 0.0
            self.command_pub.publish(target)
        else:
            self._logger.debug("Policy is not active")


def main():
    rclpy.init()
    logger = rclpy.logging.get_logger("policy_deployment")
    
    policy = SingleImagePolicy()
    
    # basic policy configuration
    policy.active = True
    policy.set_servo_command_type(ServoCommandType.Request.POSE) # set servo command type to pose
    
    rclpy.spin(policy)

    policy.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
