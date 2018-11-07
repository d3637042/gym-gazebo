import gym
import rospy
import roslaunch
import time
import numpy as np
import sensor_msgs.point_cloud2

from math import sqrt, atan2
from robotx_gazebo.msg import UsvDrive
from gym import utils, spaces
from gym_gazebo.envs import gazebo_env
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

from gym.utils import seeding
from random import randint
class GazeboRobotXLidarLocEnv(gazebo_env.GazeboEnv):

    def __init__(self):
        # Launch the simulation with the given launchfile name
        gazebo_env.GazeboEnv.__init__(self, "robotx.launch")
        self.vel_pub = rospy.Publisher('/cmd_drive', UsvDrive, queue_size=5)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        
        self.action_space = spaces.Discrete(3) #F,L,R
        self.reward_range = (-np.inf, np.inf)
        self.target_x = 50
        self.target_y = 30
        self._seed()

    def random_moves(self):
        count = randint(0,9)
        for i in range(count):
            vel_cmd = UsvDrive() 
            if randint(0, 1) == 1:
                sign = -1
            else:
                sign = 1
            vel_cmd.right = sign
            vel_cmd.left = sign
            self.vel_pub.publish(vel_cmd)
            rospy.sleep(0.5)
        rospy.sleep(5)
        #print "count", count

    def discretize_observation(self,data,new_ranges):
        discretized_ranges = np.ones(new_ranges)*5
        min_range = 3.5
        done = False
        mod = data.width/new_ranges
        point_arr = []
        angle_arr = []
        for point in sensor_msgs.point_cloud2.read_points(data, skip_nans=True):
            pt_x = point[0]
            pt_y = point[1]
            pt_z = point[2]
            distance = sqrt(pt_x**2 + pt_y**2)
            angle = atan2(pt_y, pt_x)
            if ((np.abs(pt_x) > 1.5 or pt_y > 2.5) and distance <= 30 and angle >=0 and pt_z > -1.5):
                point_arr.append(distance)      
                angle_arr.append(angle)
        for i, item in enumerate(point_arr):
            k = int(angle_arr[i]/(np.pi/new_ranges))
            if point_arr[i] == float ('Inf') or np.isinf(point_arr[i]):
                if discretized_ranges[k] < 30:
                    discretized_ranges[k] = 5
            elif np.isnan(point_arr[i]):
                if discretized_ranges[k] > 0:
                    discretized_ranges[k] = 0
            else:
                if discretized_ranges[k] > int(point_arr[i]/6):
                    discretized_ranges[k] = int(point_arr[i]/6)
            if (min_range > point_arr[i] > 0):
                done = True
        return discretized_ranges,done

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _step(self, action):
        #rospy.wait_for_service('/gazebo/unpause_physics')
        #try:
        #    self.unpause()
        #except (rospy.ServiceException) as e:
        #    print ("/gazebo/unpause_physics service call failed")

        if action == 0: #FORWARD
            vel_cmd = UsvDrive()
            vel_cmd.right = 0.5
            vel_cmd.left = 0.5
            self.vel_pub.publish(vel_cmd)
        elif action == 1: #LEFT
            vel_cmd = UsvDrive()
            vel_cmd.right = 1
            vel_cmd.left = -1
            self.vel_pub.publish(vel_cmd)
        elif action == 2: #RIGHT
            vel_cmd = UsvDrive()
            vel_cmd.right = -1
            vel_cmd.left = 1
            self.vel_pub.publish(vel_cmd)
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/velodyne_points', PointCloud2, timeout=5)
            except:
                pass

        #rospy.wait_for_service('/gazebo/pause_physics')
        #try:
        #    #resp_pause = pause.call()
        #    self.pause()
        #except (rospy.ServiceException) as e:
        #    print ("/gazebo/pause_physics service call failed")

        state,done = self.discretize_observation(data,5)
        
        loc_data = None
        while loc_data is None:
            try:
                loc_data = rospy.wait_for_message('/odometry/filtered', Odometry, timeout=5)
            except:
                pass
        dist_to_target = np.sqrt((self.target_x-loc_data.pose.pose.position.x)*(self.target_x-loc_data.pose.pose.position.x)+(self.target_y-loc_data.pose.pose.position.y)*(self.target_y-loc_data.pose.pose.position.y))
        state = np.append(state, int(dist_to_target))
        
        dist_reward = 50 - dist_to_target
     
        if dist_to_target < 5:
            done = True
            end = True
        if not done:
            if action == 0:
                reward = 10 + dist_reward
            else:
                reward = 1 + dist_reward
        else:
            if end:
                reward = 100000
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
        #rospy.wait_for_service('/gazebo/unpause_physics')
        #try:
        #    #resp_pause = pause.call()
        #    self.unpause()
        #except (rospy.ServiceException) as e:
        #    print ("/gazebo/unpause_physics service call failed")

        #read laser data
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/velodyne_points', PointCloud2, timeout=5)
            except:
                pass
        #rospy.wait_for_service('/gazebo/pause_physics')
        #try:
        #    #resp_pause = pause.call()
        #    self.pause()
        #except (rospy.ServiceException) as e:
        #    print ("/gazebo/pause_physics service call failed")

        state, done = self.discretize_observation(data,5)
        loc_data = None
        while loc_data is None:
            try:
                print "get odometry"
                loc_data = rospy.wait_for_message('/odometry/filtered', Odometry, timeout=5)
            except:
                pass
        dist_to_target = np.sqrt((self.target_x-loc_data.pose.pose.position.x)*(self.target_x-loc_data.pose.pose.position.x)+(self.target_y-loc_data.pose.pose.position.y)*(self.target_y-loc_data.pose.pose.position.y))
        state = np.append(state, int(dist_to_target))
        
        self.random_moves()
        return state, done
