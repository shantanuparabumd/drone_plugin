

import rclpy
from rclpy.node import Node
import numpy as np

from drone_plugin.msg import MotorSpeed                            # CHANGE
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64
import tf_transformations

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        
        # Subscribe to the motor_speed topic
        self.motor_speed_subscription = self.create_subscription(MotorSpeed, '/motor_speed', self.motor_speed_callback, 10)  # CHANGE
        
        # Subscribe to drone velocity
        self.drone_velocity_subscription = self.create_subscription(Twist, '/drone_velocity', self.drone_velocity_callback, 10)
        
        # Subscribe to drone pose
        self.drone_pose_subscription = self.create_subscription(PoseStamped, '/drone_pose', self.drone_pose_callback, 10)
        
        
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        
        # Publish Estimated State (Height)
        self.est_publisher_ = self.create_publisher(Float64, '/estimated_height', 10)  # CHANGE
        
        # Publish Observer State (Height)
        self.obs_publisher_ = self.create_publisher(Float64, '/observer_height', 10)
        
        
        # Drone Parameters
        self.m = 0.03 # mass of the drone
        self.g = 9.81 # gravity
        self.k_th = 1e-9 # thrust coefficient
        self.k_to = 1e-11 # torque coefficient
        self.l = 0.12 # arm length
        self.Ixx = 1e-5
        self.Iyy = 1e-5
        self.Izz = 1e-5
        self.a = 0.707 # angle between the arm and the x-axis (45 degrees)
        
        # State Variables
        self.U = np.zeros((4,1)) # control input
        self.X = np.zeros((12,1)) # state vector
        self.Y = np.zeros((12,1))
        
        self.observed_height = 0.0
        self.estimated_height = 0.0
        
        
        self.current_time = 0.0
        self.previous_time = 0.0
        self.dt = 0.0
        

    def timer_callback(self):
        A,B,C,D = self.drone_model(m=self.m, g=self.g, Ixx=self.Ixx, Iyy=self.Iyy, Izz=self.Izz)
        
        X_dot = np.dot(A, self.X) + np.dot(B, self.U)
        self.dt = self.current_time - self.previous_time
        self.X = self.X + self.dt*X_dot
        self.Y = np.dot(C, self.X) + np.dot(D, self.U)
        
        self.previous_time = self.current_time
        
        self.estimated_height = self.Y[0]
        
        msg = Float64()
        msg.data = self.estimated_height
        
        self.est_publisher_.publish(msg)
        
        msg.data = self.observed_height
        self.obs_publisher_.publish(msg)
        
        
        
        
        
        
    def drone_velocity_callback(self, msg):
        # self.X[1] = msg.linear.z
        # self.X[5] = msg.linear.x
        # self.X[9] = msg.linear.y
        # self.X[3] = msg.angular.x
        # self.X[7] = msg.angular.y
        # self.X[11] = msg.angular.z
        pass
        
    def drone_pose_callback(self, msg):
        # quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        # roll, pitch, yaw = roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)
        # self.X[0] = msg.position.z
        # self.X[4] = msg.position.x
        # self.X[8] = msg.position.y
        # self.X[2] = roll
        # self.X[6] = pitch
        # self.X[10] = yaw
        self.observed_height = msg.position.z
        self.current_time = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        



    def motor_speed_callback(self, msg):
        prop1_speed = msg.velocity[0]
        prop2_speed = msg.velocity[1]
        prop3_speed = msg.velocity[2]
        prop4_speed = msg.velocity[3]
        Ui = np.array([[prop1_speed**2], [prop2_speed**2], [prop3_speed**2], [prop4_speed**2]])
        T = np.array([self.k_th, self.k_th, self.k_th, self.k_th],
                     [self.k_th*self.l*np.cos(self.a), self.k_th*self.l*np.cos(self.a), -self.k_th*self.l*np.cos(self.a), -self.k_th*self.l*np.cos(self.a)],
                     [self.k_th*self.l*np.sin(self.a), -self.k_th*self.l*np.sin(self.a), -self.k_th*self.l*np.sin(self.a), self.k_th*self.l*np.sin(self.a)],
                     [self.k_to, -self.k_to, self.k_to, -self.k_to])
        self.U = np.dot(T, Ui)
        
        
    
    def drone_model(self,g=9.81,m=0.03,Ixx=1e-5,Iyy=1e-5,Izz=1e-5):
        A = np.array([[0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, g, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                    [0 ,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -g, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] ])
        B = np.array([[0, 0, 0, 0],
                    [1/m, 0, 0, 0],
                    [0, 0, 0, 0],
                    [0, 1/Ixx, 0, 0],
                    [0, 0, 0, 0],
                    [0, 0, 0, 0],
                    [0, 0, 0, 0],
                    [0, 0, 1/Iyy, 0],
                    [0, 0, 0, 0],
                    [0, 0, 0, 0],
                    [0, 0, 0, 0],
                    [0, 0, 0, 1/Izz]])    
        C = np.eye(12)
        D = np.zeros((12,4))
        return A, B, C, D
    
    
        
        
        
            
        
    
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


