import gym
import rospy
import roslaunch
import time
import numpy as np
import sensor_msgs.point_cloud2

from math import sqrt
from robotx_gazebo.msg import UsvDrive
from gym import utils, spaces
from gym_gazebo.envs import gazebo_env
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

from sensor_msgs.msg import PointCloud2

from gym.utils import seeding
from random import randint
class GazeboRobotXLidarEnv(gazebo_env.GazeboEnv):

    def __init__(self):
        # Launch the simulation with the given launchfile name
        gazebo_env.GazeboEnv.__init__(self, "robotx.launch")
        self.vel_pub = rospy.Publisher('/cmd_drive', UsvDrive, queue_size=5)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        
        self.action_space = spaces.Discrete(3) #F,L,R
        self.reward_range = (-np.inf, np.inf)

        self._seed()

    def random_moves(self):
        count = randint(0,9)
        for i in range(count):
            vel_cmd = UsvDrive()
            vel_cmd.right = -500
            vel_cmd.left = 500
            self.vel_pub.publish(vel_cmd)
            rospy.sleep(0.5)
        print "count", count

    def discretize_observation(self,data,new_ranges):
        discretized_ranges = []
        min_range = 3.5
        done = False
        mod = data.width/new_ranges
        point_arr = []

        for point in sensor_msgs.point_cloud2.read_points(data, skip_nans=True):
            pt_x = point[0]
            pt_y = point[1]
            distance = sqrt(pt_x**2 + pt_y**2)
            
            if (distance>=3.5 and distance<=20):
                point_arr.append(distance)      

        for i, item in enumerate(point_arr):
            if (i%mod==0):
                if point_arr[i] == float ('Inf') or np.isinf(point_arr[i]):
                    discretized_ranges.append(10)
                elif np.isnan(point_arr[i]):
                    discretized_ranges.append(0)
                else:
                    discretized_ranges.append(int(point_arr[i]/2))
            if (min_range > point_arr[i] > 0):
                done = True
        return discretized_ranges,done

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _step(self, action):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        if action == 0: #FORWARD
            vel_cmd = UsvDrive()
            vel_cmd.right = 500
            vel_cmd.left = 500
            self.vel_pub.publish(vel_cmd)
        elif action == 1: #LEFT
            vel_cmd = UsvDrive()
            vel_cmd.right = 500
            vel_cmd.left = -500
            self.vel_pub.publish(vel_cmd)
        elif action == 2: #RIGHT
            vel_cmd = UsvDrive()
            vel_cmd.right = -500
            vel_cmd.left = 500
            self.vel_pub.publish(vel_cmd)
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/velodyne_points', PointCloud2, timeout=5)
            except:
                pass

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        state,done = self.discretize_observation(data,5)

        if not done:
            if action == 0:
                reward = 10
            else:
                reward = 0
        else:
            reward = -200

        return state, reward, done, {}

    def _reset(self):

        # Resets the state of the environment and returns an initial observation.
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            #reset_proxy.call()
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/reset_simulation service call failed")

        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            #resp_pause = pause.call()
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        #read laser data
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/velodyne_points', PointCloud2, timeout=5)
            except:
                pass
        self.random_moves()
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        state = self.discretize_observation(data,5)


        return state
