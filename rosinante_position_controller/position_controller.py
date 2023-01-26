# /*
# * Copyright (C) 2021 Alexander Junk <dev@junk.technology>
# * 
# * This program is free software: you can redistribute it and/or modify it 
# * under the terms of the GNU Lesser General Public License as published 
# * by the Free Software Foundation, either version 3 of the License, or 
# * (at your option) any later version.
# * 
# * This program is distributed in the hope that it will be useful,
# * but WITHOUT ANY WARRANTY; 
# * without even the implied warranty of MERCHANTABILITY or FITNESS
# * FOR A PARTICULAR PURPOSE. 
# * See the GNU Lesser General Public License for more details.
# * 
# * You should have received a copy of the GNU Lesser General Public License 
# * along with this program. If not, see <https://www.gnu.org/licenses/>. 
# *
# */

import rclpy
from rclpy.node import Node
from rclpy.node import LifecycleNode

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from math import sqrt
from math import atan2
import math

from rclpy.executors import MultiThreadedExecutor

from rclpy.action import ActionServer
from rosinante_position_controller_interfaces.action import GoToPose

import PyKDL

def transform_to_kdl(t):
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                 t.transform.rotation.z, t.transform.rotation.w),
                       PyKDL.Vector(t.transform.translation.x,
                                    t.transform.translation.y,
                                    t.transform.translation.z))


# PoseStamped
def do_transform_pose(pose, transform):
    f = transform_to_kdl(transform) * PyKDL.Frame(PyKDL.Rotation.Quaternion(pose.pose.orientation.x, pose.pose.orientation.y,
                                                                          pose.pose.orientation.z, pose.pose.orientation.w),
                                                PyKDL.Vector(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z))
    res = PoseStamped()
    res.pose.position.x = f.p[0]
    res.pose.position.y = f.p[1]
    res.pose.position.z = f.p[2]
    (res.pose.orientation.x, res.pose.orientation.y, res.pose.orientation.z, res.pose.orientation.w) = f.M.GetQuaternion()
    res.header = transform.header
    return res


def euler_from_quaternion(x, y, z, w):
		"""
		Convert a quaternion into euler angles (roll, pitch, yaw)
		roll is rotation around x in radians (counterclockwise)
		pitch is rotation around y in radians (counterclockwise)
		yaw is rotation around z in radians (counterclockwise)
		"""
		t0 = +2.0 * (w * x + y * z)
		t1 = +1.0 - 2.0 * (x * x + y * y)
		roll_x = math.atan2(t0, t1)
    
		t2 = +2.0 * (w * y - z * x)
		t2 = +1.0 if t2 > +1.0 else t2
		t2 = -1.0 if t2 < -1.0 else t2
		pitch_y = math.asin(t2)
    
		t3 = +2.0 * (w * z + x * y)
		t4 = +1.0 - 2.0 * (y * y + z * z)
		yaw_z = math.atan2(t3, t4)
    
		return roll_x, pitch_y, yaw_z # in radians

class BasicPositionController(Node):

    def __init__(self):
        super().__init__('basic_position_controller')

        self.declare_parameter('target_frame', 'map')
        self.target_frame = self.get_parameter(
            'target_frame').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.025  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback_)
        self.i = 0
        
        self.status_timer = self.create_timer(0.5, self.status_timer_callback_)
        self.reference_frame_publisher_ = self.create_publisher(String, f'{self.get_name()}/reference_frame', 1)
        
        self.target_pose_ = PoseStamped()
        self.pose_subscriber_ = self.create_subscription(
            PoseStamped, 'target_pose',
            self.pose_subscriber_callback_,
            10
        )
        self.pose_subscriber_

        self.target_active_ = False
        self.target_hold_ = False

        self.current_distance_remaining_ = 0.0

        self._action_server = ActionServer(
            self,
            GoToPose,
            'go_to_pose',
            self.go_to_pose_cb)

    def go_to_pose_cb(self, goal_handle):
        self.get_logger().info('Executing goal...')

        self.target_pose_ = goal_handle.request.target_pose
        # self.target_pose_.header.frame_id = self.target_frame
        self.target_frame = self.target_pose_.header.frame_id
        self.target_active_ = True

        feedback_msg = GoToPose.Feedback()
        feedback_msg.distance = self.current_distance_remaining_

        import time
        while(self.target_active_):
            time.sleep(0.1)
            #self.timer_callback_()
            feedback_msg.distance = self.current_distance_remaining_
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()

        result = GoToPose.Result()
        result.success = True
        result.message = ""
        return result

    def status_timer_callback_(self):
        msg = String()
        msg.data = self.target_frame

        self.reference_frame_publisher_.publish(msg)

    def timer_callback_(self):
        #msg = Twist()
        #msg.linear.x = 0.02 * (-1)**(self.i % 2) 
        #self.cmd_publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg)
        #self.i += 1

        #self.cmd_publisher_.publish(msg)

        from_frame_rel = self.target_frame
        to_frame_rel = 'base_link'

        

        if not self.target_active_:
            return

        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                now)

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        pt = do_transform_pose(self.target_pose_, trans)

        abs_val = sqrt(pt.pose.position.x**2 + pt.pose.position.y**2)

        pt.pose.position.x /= abs_val
        pt.pose.position.y /= abs_val

        msg = Twist()
        #scale_rotation_rate = 0.02
        scale_rotation_rate = 0.2
        
        q = pt.pose.orientation

        #msg.angular.z = scale_rotation_rate * pt.pose.orientation.z
        roll, pitch, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

        abs_yaw = abs(yaw)
        yaw /= abs_yaw
        
        if(abs_yaw < 2*3.1415/360/2):
            scale_rotation_rate = 0.00
        elif(abs_yaw < 2*3.1415/360*1):
            scale_rotation_rate = 0.005
        
        elif(abs_yaw < 2*3.1415/360*5):
            scale_rotation_rate = 0.05

        if(yaw > 0):
            msg.angular.z = scale_rotation_rate
        elif(yaw < 0):
            msg.angular.z = -1 * scale_rotation_rate

        

        #scale_forward_speed = 0.03
        scale_forward_speed = 0.3
        
        #if(abs_val < 0.001):
        
        if(abs_val < 0.01):
            scale_forward_speed = 0.000
        elif(abs_val < 0.05):
            scale_forward_speed = 0.05   
        elif(abs_val < 0.1):
            scale_forward_speed = 0.1
        elif(abs_val < 0.3):
            scale_forward_speed = 0.2
        else:
            scale_forward_speed = 0.3

        msg.linear.x = scale_forward_speed * pt.pose.position.x
        msg.linear.y = scale_forward_speed * pt.pose.position.y

        self.current_distance_remaining_ = abs_val

        self.cmd_publisher_.publish(msg)

        if not self.target_hold_ and scale_forward_speed == 0.0 and scale_rotation_rate == 0.0:
            self.target_active_ = False

    def pose_subscriber_callback_(self, msg):
        self.get_logger().info('Got: "%s"' % msg)
        self.target_pose_ = msg
        # self.target_pose_.header.frame_id = self.target_frame
        self.target_frame = self.target_pose_.header.frame_id
        self.target_active_ = True

def main(args=None):
    rclpy.init(args=args)

    basic_position_controller = BasicPositionController()


     # MultiThreadedExecutor executes callbacks with a thread pool. If num_threads is not
    # specified then num_threads will be multiprocessing.cpu_count() if it is implemented.
    # Otherwise it will use a single thread. This executor will allow callbacks to happen in
    # parallel, however the MutuallyExclusiveCallbackGroup in DoubleTalker will only allow its
    # callbacks to be executed one at a time. The callbacks in Listener are free to execute in
    # parallel to the ones in DoubleTalker however.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(basic_position_controller)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        listener.destroy_node()
        talker.destroy_node()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
