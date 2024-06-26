import rclpy
from rclpy.node import Node

import threading
import time

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
from rclpy.callback_groups import ReentrantCallbackGroup

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rosinante_position_controller_interfaces.action import GoToPose

import PyKDL

from simple_pid import PID


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

        self.get_logger().info('!Reading parameters.')

        self.declare_parameter('target_frame', 'odom')
        self.target_frame = self.get_parameter(
            'target_frame').get_parameter_value().string_value

        self.get_logger().info('Initializing TF2 listener.')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('Preparing publishers and subscribers.')

        self.state_publisher_ = self.create_publisher(String, '/state', 10)

        self.cmd_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback_)
        self.i = 0
        
        self.get_logger().info('Setting up timers.')
 
        self.status_timer = self.create_timer(0.5, self.status_timer_callback_)
 
        self.reference_frame_publisher_ = self.create_publisher(String, f'{self.get_name()}/reference_frame', 1)


        self.target_pose_ = PoseStamped()
        self.pose_subscriber_ = self.create_subscription(
            PoseStamped, 'target_pose',
            self.pose_subscriber_callback_,
            10
        )

        self.target_active_ = False
        self.target_hold_ = False

        self.current_distance_remaining_ = 0.0

        self.get_logger().info('Setting up action server.')


        self._goal_handle = None
        self._goal_lock = threading.Lock()
        
        self._action_server = ActionServer(
            self,
            GoToPose,
            '/go_to_pose',
            self.go_to_pose_cb
            #goal_callback = self.goal_callback,
            #cancel_callback = self.cancel_callback,
            #handle_accepted_callback=self.handle_accepted_callback
            )

        self.idle_ = True


        self.xy_pid = PID(20, 0.02, 0.5, setpoint=0)
        self.xy_pid.output_limits = (-1, 1) 
        self.xy_pid.set_auto_mode(False)

        #self.xy_pid.proportional_on_measurement = True

        self.get_logger().info('Position controller initialized!')

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        self.get_logger().info('Accepted goal request')

        goal_handle.execute()

    def go_to_pose_cb(self, goal_handle):
        self.get_logger().info('Executing goal...')

        self.target_pose_ = goal_handle.request.target_pose
        #self.target_pose_.header.frame_id = self.target_frame
        #self.target_pose_.header.frame_id = 'odom'
        self.target_frame = goal_handle.request.target_pose.header.frame_id
        self.target_active_ = True

        feedback_msg = GoToPose.Feedback()
        feedback_msg.distance = self.current_distance_remaining_

        while(self.target_active_):
            time.sleep(0.1)
            #self.timer_callback_()
            #feedback_msg.distance = self.current_distance_remaining_
            #goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()

        self.get_logger().info('Goal reached...')

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
        #from_frame_rel = 'map'
        #from_frame_rel = 'odom'
        to_frame_rel = 'base_link'

        state_msg = String()

        if not self.target_active_ and self.idle_:
            return
        elif not self.target_active_ and not self.idle_:
            self.idle_ = True
            state_msg.data = "idle"
            self.state_publisher_.publish(state_msg)
            return
            
        elif self.target_active_ and self.idle_:
            self.idle_ = False
            state_msg.data = "running"
            self.state_publisher_.publish(state_msg)
            self.xy_pid.set_auto_mode(True)

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

        self.get_logger().info(".")

        pt = do_transform_pose(self.target_pose_, trans)

        abs_val = sqrt(pt.pose.position.x**2 + pt.pose.position.y**2)

        pt.pose.position.x /= abs_val
        pt.pose.position.y /= abs_val

        msg = Twist()
        #scale_rotation_rate = 0.02
        scale_rotation_rate = 0.05
        
        q = pt.pose.orientation

        #msg.angular.z = scale_rotation_rate * pt.pose.orientation.z
        roll, pitch, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

        abs_yaw = abs(yaw)
        yaw /= abs_yaw
        
        if(abs_yaw < 2*3.1415/360*0.1):
            scale_rotation_rate = 0.00
        elif(abs_yaw < 2*3.1415/360*0.5):
            scale_rotation_rate = 0.1
        elif(abs_yaw < 2*3.1415/360*2):
            scale_rotation_rate = 0.2
        elif(abs_yaw < 2*3.1415/360*10):
            scale_rotation_rate = 0.2
        
        elif(abs_yaw < 2*3.1415/360*20):
            scale_rotation_rate = 0.2

        scale_rotation_rate = 0.0
        
        if(yaw > 0):
            msg.angular.z = scale_rotation_rate
        elif(yaw < 0):
            msg.angular.z = -1 * scale_rotation_rate
        else:
            msg.angular.z = 0
        
        # Safe working speed!
        #scale_forward_speed = 0.03
        #scale_forward_speed = 1.0
        scale_forward_speed = 0.15

        #msg.linear.x = scale_forward_speed * pt.pose.position.x
        #msg.linear.y = scale_forward_speed * pt.pose.position.y
        
        if abs_val <= 0.01:
            abs_val = 0

        v = self.xy_pid(abs_val)
        p, i, d = self.xy_pid.components
        print(f'{p} ${i} ${d}')
        print(v)

      

        msg.linear.x =  abs(v) *  scale_forward_speed * pt.pose.position.x
        msg.linear.y = abs(v) *  scale_forward_speed * pt.pose.position.y
        

        self.current_distance_remaining_ = abs_val

        self.cmd_publisher_.publish(msg)

        #if not self.target_hold_ and scale_forward_speed == 0.0 and scale_rotation_rate == 0.0:
        if not self.target_hold_ and abs_val == 0:
            self.target_active_ = False
            self.xy_pid.set_auto_mode(False)

    def pose_subscriber_callback_(self, msg):
        self.get_logger().info('Got: "%s"' % msg)
        self.target_pose_ = msg
        #self.target_pose_.header.frame_id = self.target_frame
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
    executor = MultiThreadedExecutor()
    #executor.add_node(basic_position_controller)
    try:
        rclpy.spin(basic_position_controller, executor=executor)
    finally:
        executor.shutdown()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    basic_position_controller.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
