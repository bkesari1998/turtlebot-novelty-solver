#!/usr/bin/env python

# ON TURTLEBOT
# roslaunch coordinate_navigation turtlebot.launch
# roslaunch coordinate_navigation plan_1_start.launch waypoints_file:=plan_1_curtain_novelty_waypoints.yaml

# ON REMOTE
# roslaunch coordinate_navigation remote.launch

# ON TURTLEBOT
# rosrun coordinate_navigation manager.py

import pickle
import math
import rospy
import os
import numpy as np
from coffee_bot_srvs.srv import Action, Move, PrimitiveAction
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

from kobuki_msgs.msg import BumperEvent

import actionlib
from turtlebot_actions.msg import TurtlebotMoveAction, TurtlebotMoveGoal


from nav_msgs.srv import GetPlan
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, Empty

from tf.transformations import quaternion_multiply, quaternion_inverse

from learn_exec import *


class Manager(object):
    def __init__(self):

        rospy.init_node("manager")

        rospy.wait_for_service("/action_executor")
        rospy.wait_for_service("/move_base/make_plan")
        rospy.loginfo("action_executor service active")

        self.object_list = rospy.get_param("object_list")
        self.waypoint_list = rospy.get_param("waypoint_list")
        self.waypoint_loc = rospy.get_param("waypoints")

        # Get agent high level state information
        self.agent_state = rospy.get_param("agents/turtlebot")
        self.update_state_subscriber = rospy.Subscriber(
            "update_state", Bool, self.update_state_handler
        )
        self.state_confirmer = rospy.ServiceProxy("confirm_state", Trigger)

        # Get plan from plan file
        # TODO

        # PDDL problem generator
        self.pddl_goal = ["facing desk_1"]
        self.make_plan_client = rospy.ServiceProxy("move_base/make_plan", GetPlan)

        # Action clients
        self.move_client = rospy.ServiceProxy("move", Move)
        self.primitive_move_client = rospy.ServiceProxy(
            "primitive_move_actions", PrimitiveAction
        )
        self.action_executor_client = rospy.ServiceProxy("action_executor", Action)

        # Assign primitive move actions
        self.primitive_moves = {"forward": 0, "turn_cc": 1, "turn_c": 2}
        self.primitive_moves_list = [
            ["move", "forward"],
            ["move", "turn_cc"],
            ["move", "turn_c"],
        ]

        # Clear costmaps service
        self.clear_costmaps = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)

        # Get the goal state
        self.reward_function = rospy.get_param(
            "reward"
        )  # list of reward states for all the failed operator.

        # Instantiate bumper listner
        bumper_listner = rospy.Subscriber(
            "/mobile_base/events/bumper", BumperEvent, self.bumper_handler
        )
        self.bumper_counter = 0
        self.last_bumper_time = rospy.Time.now()

        # Initialize action client
        self.move_action_client = actionlib.SimpleActionClient(
            "turtlebot_move", TurtlebotMoveAction
        )
        self.move_action_client.wait_for_server()

        # Get model information
        self.load_model_flag = rospy.get_param("load_model_flag")
        self.failed_operator_name = rospy.get_param("failed_operator_name")
        self.lfd_flag = rospy.get_param("lfd_flag")

        # Other class info
        self.episodes = 0
        self.learner = None
        self.R = []  # storing rewards per episode
        self.Dones = []  # storing goal completion per episode
        self.Steps = []  # storing the number of steps in each episode
        self.Eps = []  # storing the epsilons in the list for each episode
        self.data = [self.R, self.Dones, self.Steps, self.Eps]

        # Load model if flag is true
        if self.load_model_flag:
            self.load_model()

    def main(self):

        # Get initial observation
        self.update_state_handler((Bool(True)))
        init_obs = np.array(self.build_learner_state())

        # Instantiate learner
        self.learner = Learner(
            self.failed_operator_name,
            init_obs,
            self.primitive_moves,
            episode_num=self.episodes,
            load_model_flag=self.load_model_flag,
        )
        self.learner.agent.set_explore_epsilon(params.MAX_EPSILON)

        for episode in range(params.MAX_EPISODES):

            # Episode counters
            self.episodes += 1
            self.timesteps = 0
            self.reward = 0
            self.done = 0

            # set the explore epsilon
            self.epsilon = params.MIN_EPSILON + (
                params.MAX_EPSILON - params.MIN_EPSILON
            ) * math.exp(-params.LAMBDA * episode)
            self.learner.agent._explore_eps = self.epsilon

            # Build starting state for episode
            self.update_state_handler((Bool(True)))
            obs = np.array(self.build_learner_state())

            # Running timesteps for episode
            while True:
                self.reward_counter = 0
                # Check for bumper press
                if self.bumper_counter >= 5:
                    rospy.loginfo(
                        "Turtlebot has run into an object too many times consecutively.  Resetting..."
                    )
                    if len(self.learner.agent._drs) > 0:
                        self.learner.agent._drs.pop()  # hack since we might have given a reward already before
                        # TODO
                        # Handle situation where _drs is 0
                    self.end_episode(reward=-5)
                    break  # out of while loop

                # Get the action from the network and execute
                if self.lfd_flag:
                    # lfd
                    action_ = raw_input("Action number: ")
                    while action_ not in ["0", "1", "2"]:
                        action_ = raw_input("Action number: ")
                    learner_action = self.learner.get_action(obs, False, int(action_))
                else:
                    # Not lfd
                    learner_action = self.learner.get_action(obs)

                # Execute action
                status = self.action_executor_client(
                    self.primitive_moves_list[learner_action]
                )

                # Action executor client will return False if bumper is pressed to indicate failed action
                if status:
                    self.bumper_counter = 0
                else:
                    self.bumper_counter += 1

                # Update state
                self.update_state_handler((Bool(True)))
                self.timesteps += 1
                obs = np.array(self.build_learner_state())

                # Check for goal state
                if (
                    self.agent_state["at"]
                    == self.reward_function[self.failed_operator_name]["at"]
                    and self.agent_state["facing"]
                    == self.reward_function[self.failed_operator_name]["facing"]
                ):
                    rospy.loginfo("Goal state reached! Resetting")
                    self.done = 1
                    self.end_episode(reward=1000)
                    break

                # Negative reward for each step
                self.learner.agent.give_reward(-1)
                self.reward -= 1

                # Max steps reached
                if self.timesteps >= params.MAX_TIMESTEPS:
                    rospy.loginfo("Max quota reached. Resetting.")
                    self.end_episode()
                    break

            self.save_logger()
            self.print_stuff()

        # Save model and data
        self.learner.agent.save_model(self.failed_operator_name, self.episodes)
        self.save_data()

    def load_model(self):
        file_names = os.listdir(
            "/home/mulip/catkin_ws/src/coffee-bot/coordinate_navigation/scripts/models"
        )
        ep_nums = []
        if file_names:
            for file_name in file_names:
                ep_num = int(file_name.split(".")[0].split("_")[-1])  # episode number
                ep_nums.append(ep_num)
            self.episodes = max(ep_nums)

    def end_episode(self, reward=None):
        if reward != None:
            self.learner.agent.give_reward(reward)
            self.reward += reward
        self.learner.agent.finish_episode()
        self.learner.agent.update_parameters()

        episode_reset = "n"
        while episode_reset != "y":
            episode_reset = raw_input("Reset to starting position? ")

        self.clear_costmaps()
        self.move_client("exploration_reset")  # reset to begin state

    def print_stuff(self):
        rospy.loginfo(
            "Episode--> %s Reward-->> %s Steps--> %s",
            self.episodes,
            self.reward,
            self.timesteps,
        )

    def save_data(self):
        path = (
            "/home/mulip/catkin_ws/src/coffee-bot/coordinate_navigation/scripts/data/data"
            + str(self.episodes)
            + ".pickle"
        )
        memory = {"data": self.data}
        f = open(path, "wb")
        pickle.dump(memory, f)
        f.close()

    def save_logger(self):
        self.Steps.append(self.timesteps)
        self.R.append(self.reward)
        self.Eps.append(self.epsilon)
        self.Dones.append(self.done)

    def bumper_handler(self, msg):
        if msg.state == BumperEvent.PRESSED:
            self.move_action_client.cancel_all_goals()
            if rospy.Time.now() - self.last_bumper_time < rospy.Duration(2.0):
                self.last_bump_time = rospy.Time.now()
                self.bumper_counter += 1
            else:
                self.last_bumper_time = rospy.Time.now()
                self.bumper_counter = 0

    def execute_plan(self, plan):

        for action in plan:
            res = self.action_executor_client(action)

            if not res.success:
                rospy.loginfo(action[0])
                rospy.loginfo(res.message)
                return [False, action]

        return [True, []]

    def read_plan(self, plan_file_path):

        f = open(plan_file_path, "r")

        # Get lines of file
        lines = f.readlines()

        # Only get plan lines
        plan_lines = lines[3:-3]

        # Remove timestamp from plan lines
        for i in range(len(plan_lines)):

            plan_lines[i] = plan_lines[i][
                plan_lines[i].index("(") + 1 : plan_lines[i].index(")")
            ]

        # Return plan, with each action split by space char
        plan = []
        for line in plan_lines:
            plan.append(line.split(" "))

        f.close()

        return plan

    def build_learner_state(self):

        learner_state = [0] * 3

        if "lab" in self.agent_state["at"]:
            learner_state[0] = 1
        elif "hallway" in self.agent_state["at"]:
            learner_state[1] = 1
        else:
            learner_state[2] = 1

        facing_indecies = dict(zip(self.object_list, np.arange(len(self.object_list))))
        facing_index = facing_indecies[self.agent_state["facing"][0]]
        facing_state = [0] * len(self.object_list)
        facing_state[facing_index] = 1

        learner_state = learner_state + facing_state

        # Add distances and orientation to all static objects, add true false for
        odom_pose_with_cov_stamped = rospy.wait_for_message(
            "amcl_pose", PoseWithCovarianceStamped, rospy.Duration(1)
        )
        odom_pose = odom_pose_with_cov_stamped.pose.pose

        for waypoint in self.waypoint_list:
            pose = self.waypoint_loc[waypoint]

            dxdy = []
            # append x and y dist to list
            dxdy.append(pose[0][0] - odom_pose.position.x)
            dxdy.append(pose[0][1] - odom_pose.position.y)

            rot = []
            # append orientation info
            q_1_inverse = quaternion_inverse(pose[1])
            q_0 = [
                odom_pose.orientation.x,
                odom_pose.orientation.y,
                odom_pose.orientation.z,
                odom_pose.orientation.w,
            ]
            rel_orientation = quaternion_multiply(q_0, q_1_inverse)
            rot.append(rel_orientation[2])
            rot.append(rel_orientation[3])

            # Add dxdy and rot to learner state
            learner_state = learner_state + dxdy + rot
            if waypoint != "novel_object":  # Don't need plan_exists to novel object
                odom_pose_stamped = self.pose_with_covariance_stamed_to_pose_stamped(
                    odom_pose_with_cov_stamped
                )
                waypoint_pose_stamped = self.waypoint_to_pose_stamped(pose)

                plan = self.make_plan_client(
                    odom_pose_stamped, waypoint_pose_stamped, 0.25
                )

                if len(plan.plan.poses) > 0:
                    learner_state.append(1)
                else:
                    learner_state.append(0)

        return learner_state

    def pose_with_covariance_stamed_to_pose_stamped(self, pose):
        if not isinstance(pose, PoseWithCovarianceStamped):
            return PoseStamped()

        pose_stamped = PoseStamped()
        pose_stamped.header = pose.header
        pose_stamped.pose = pose.pose.pose

        return pose_stamped

    def waypoint_to_pose_stamped(self, waypoint):

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.header.stamp = rospy.Time.now()

        pose_stamped.pose.position.x = waypoint[0][0]
        pose_stamped.pose.position.y = waypoint[0][1]

        pose_stamped.pose.orientation.z = waypoint[1][2]
        pose_stamped.pose.orientation.w = waypoint[1][3]

        return pose_stamped

    def generate_plan(self, goal_state):

        pass

    def instantiate_learner(self, learner_state, action):

        pass

    def update_state_handler(self, msg):
        if msg.data == True:
            self.agent_state = rospy.get_param("agents/turtlebot")


if __name__ == "__main__":
    manager = Manager()
    manager.main()
