#!/usr/bin/env python

import math
import rospy
import os
import csv
import time
import numpy as np
from coffee_bot_srvs.srv import Action, Move, PrimitiveAction
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

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

        self.object_list = rospy.get_param("object_list")
        self.waypoint_list = rospy.get_param("waypoint_list")
        self.waypoint_loc = rospy.get_param("waypoints")

        # Get agent high level state information
        self.agent_state = rospy.get_param("agents/turtlebot")
        self.update_state_subscriber = rospy.Subscriber(
            "update_state", Bool, self.update_state_handler
        )
        self.state_confirmer = rospy.ServiceProxy("confirm_state", Trigger)


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

        self.bumper_counter = 0
        self.last_bumper_time = rospy.Time.now()

        self.use_plan = rospy.get_param("use_plan_flag")
        self.continue_from = rospy.get_param("continue_from")
        self.plan = None
        if self.use_plan:
            plan_path = rospy.get_param("plan_path")
            self.plan = self.read_plan(plan_path)

        # Get model information
        self.load_model_flag = rospy.get_param("load_model_flag")
        
        self.model_path = rospy.get_param("model_path")
        self.data_path = rospy.get_param("data_path")
        self.trial_num = rospy.get_param("trail_number")

        # Other class info
        self.episodes = 0
        self.learner = None

        # Load model if flag is true, for testing of rl learner only
        self.failed_operator_name = ""
        self.lfd_flag = False
        if not self.use_plan and self.load_model_flag:
            self.load_model()
            self.failed_operator_name = rospy.get_param("failed_operator_name")
            self.lfd_flag = rospy.get_param("lfd_flag")

    def main(self):

        if not self.use_plan:
            self.learn_executor()
        else:
            status = False
            # Loop until all actions in a plan are successful
            while not status:
                # Execute plan
                status, failed_ops = self.execute_plan(self.plan)
                self.failed_operator_name = "_".join(failed_ops)

                # Plan failed
                if status == False:
                    goal_reached = self.rapid_learn(failed_ops)
                    if not goal_reached:
                        rospy.loginfo("RAPid Learn Failed :(")
                        return
                    else:
                        self.resume_plan(failed_ops)
                        continue
            
        rospy.loginfo("Goal reached!")


    def rapid_learn(self, failed_ops):
            
        # Check if action executor exists (is directory empty or not)
        if not os.listdir(self.model_path + os.sep + self.failed_operator_name + os.sep + "trial_" + str(self.trial_num)):    
            # List is empty, start learner from scratc
            self.load_model_flag = False
        else:
            # Executor exists
            self.load_model_flag = True
            self.load_model()

        return self.learn_executor()

    def resume_plan(self, failed_ops):
        next_action = self.continue_from[self.failed_operator_name]

        if next_action == "None":
            self.plan = []
            return

        next_action_index = self.plan.index(next_action)
        self.plan = self.plan[next_action_index:]

    def learn_executor(self):
        
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
            log_dir = self.model_path,
            trial_number = self.trial_num
        )
        self.learner.agent.set_explore_epsilon(params.MAX_EPSILON)

        for episode in range(params.MAX_EPISODES):
            # Starting episode time
            start_time = time.time()

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
                    self.bumper_counter = 0
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
                if status.success:
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
                    rospy.loginfo("Goal state reached!")
                    self.done = 1
                    self.end_episode(reward=1000)
                    self.learner.agent.save_model(self.failed_operator_name, self.episodes, path_to_save = self.model_path + os.sep + self.failed_operator_name + os.sep + "trial_" + str(self.trial_num))
                    
                    self.elapsed_time = time.time() - start_time

                    self.print_stuff()
                    self.save_to_file()
                    
                    # Return to plan
                    if self.use_plan:
                        return True
                    break


                # reward for each timestep otherwise
                self.learner.agent.give_reward(-1)
                self.reward -= 1

                # Check for goal load_model_flaged    def read_plan(self, plan_file_path):
                if self.timesteps >= params.MAX_TIMESTEPS:
                    rospy.loginfo("Max quota reached. Resetting.")
                    self.end_episode()
                    break
                
            if episode % params.SAVE_EVERY == 0:
              
                # Save model and data
                self.learner.agent.save_model(self.failed_operator_name, self.episodes, path_to_save=self.model_path + os.sep + self.failed_operator_name + os.sep + "trial_" + str(self.trial_num))

            self.elapsed_time = time.time() - start_time

            self.print_stuff()
            self.save_to_file()

        return False

    def load_model(self):
        file_names = os.listdir(self.model_path + os.sep + self.failed_operator_name + os.sep + "trial_" + str(self.trial_num))
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

        if self.use_plan and reward > 0:
            return
        
        episode_reset = "n"
        while episode_reset != "y":
            episode_reset = raw_input("Reset to starting position? ")

            self.clear_costmaps()
            self.move_client(self.failed_operator_name)  # reset to begin state

    def print_stuff(self):
        rospy.loginfo(
            "Episode--> %s Reward-->> %s Steps--> %s Elapsed Time --> %s",
            self.episodes,
            self.reward,
            self.timesteps,
            self.elapsed_time
        )


    def save_to_file(self):
        data = [self.episodes, self.timesteps, self.reward, self.epsilon, self.elapsed_time]
        db_file_name = self.data_path + os.sep + self.failed_operator_name + os.sep + "trial_" + str(self.trial_num) + os.sep + "results.csv"
        with open(db_file_name, 'a') as f:
            writer = csv.writer(f)
            writer.writerow(data)

    def execute_plan(self, plan):

        for action in plan:
            res = self.action_executor_client(action)

            if not res.success:
                rospy.loginfo("Action failed: %s" % "_".join(action))
                return False, action

        return True, []

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

    def update_state_handler(self, msg):
        if msg.data == True:
            self.agent_state = rospy.get_param("agents/turtlebot")


if __name__ == "__main__":
    manager = Manager()
    manager.main()
