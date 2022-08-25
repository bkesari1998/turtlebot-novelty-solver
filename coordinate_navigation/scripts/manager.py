#!/usr/bin/env python

import rospy
import numpy as np
from coffee_bot_srvs.srv import Action, Goal, Move, PrimitiveAction
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

from kobuki_msgs.msg import BumperEvent

from nav_msgs.srv import GetPlan
from std_msgs.msg import Bool

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
        self.update_state_subscriber = rospy.Subscriber("update_state", Bool, self.update_state_handler)

        # Get plan from plan file
        self.plan_file_path = rospy.get_param("plan_file")
        plan = self.read_plan(self.plan_file_path)

        self.action_executor_client = rospy.ServiceProxy("action_executor", Action)
        self.pddl_problem_gen_client = rospy.ServiceProxy("problem_gen", Goal)
        self.pddl_goal = ["facing desk_1"]
        self.make_plan_client = rospy.ServiceProxy("move_base/make_plan", GetPlan)
        self.move_client = rospy.ServiceProxy("move", Move)
        self.primitive_move_client = rospy.ServiceProxy("primitive_move_actions", PrimitiveAction)

        self.primitive_moves = {"forward": 0, "backward": 1, "turn_cc": 2, "turn_c": 3}
        self.primitive_moves_list = [["move", "forward"], ["move", "backward"], ["move", "turn_cc"], ["move", "turn_c"]]
        
        # Bumper
        self.bumper_pressed = 0

        self.episodes = 0
        self.steps = 0

        # Go until goal state reached
        plan_success = [False, []]
        while not plan_success[0]:

            plan_success = self.execute_plan(plan)

            if not plan_success[0]:
                # Update State
                self.update_state_handler(Bool(True))
                init_obs = np.array(self.build_learner_state())
                
                failed_operator_name = "_".join(plan_success[1])
                learner = Learner(failed_operator_name, init_obs, self.primitive_moves)
                learner.agent.set_explore_epsilon(0.2)

                # Instantiate bumper listner
                bumper_listner = rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, self.bumper_handler)

                while True:
                    self.steps += 1
                    rospy.loginfo("step num: %d" % self.steps)
                    obs = np.array(self.build_learner_state())

                
                    action_index = learner.get_action(obs, False)
                    rospy.loginfo(action_index)
                    self.action_executor_client(self.primitive_moves_list[action_index])
                    # Update State
                    self.update_state_handler(Bool(True))

                    # if self.episodes == 2:
                    #     self.agent_state["at"] = ["hallway"]

                    if self.episodes == 2:
                        learner.agent.give_reward(1000)
                        learner.agent.finish_episode()
                        learner.agent.update_parameters()
                        learner.agent.save_model(failed_operator_name)
                        rospy.loginfo("Steps = %d" % self.steps)
                        bumper_listner.unregister()
                        self.episodes += 1
                        rospy.loginfo("episode: %d" % self.episodes)
                        return

                    # Checking for
                    if obs[-1] == 1:
                        learner.agent.give_reward(-10)
                    else:
                        learner.agent.give_reward(-1)   


                    if self.steps == 5:
                        learner.agent.finish_episode()
                        learner.agent.update_parameters()
                        rospy.loginfo("Steps = %d" % self.steps)

                        self.move_client("exploration_reset")
                        self.steps = 0
                        self.episodes += 1
                        rospy.loginfo("episode: %d" % self.episodes)
                        continue

                    rospy.loginfo("Executed primitive action")

                bumper_listner.unregister()
    
    # def _parse_planner_output(self, planner_output):
    #     ff_plan = re.findall(r"\d+?: (.+)", planner_output.lower()) # matches the string to find the plan bit from the ffmetric output.


    def bumper_handler(self, msg):

        if msg.state == BumperEvent.PRESSED:
            self.bumper_pressed = 1
            self.primitive_move_client("backward")
        elif msg.state == BumperEvent.RELEASED:
            self.bumper_pressed = 0


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
        plan_lines = lines[3: -3]

        # Remove timestamp from plan lines
        for i in range(len(plan_lines)):

            plan_lines[i] = plan_lines[i][plan_lines[i].index("(") + 1: plan_lines[i].index(")")]

        # Return plan, with each action split by space char
        plan = []
        for line in plan_lines:
            plan.append(line.split(" "))

        f.close()

        return plan

    def build_learner_state(self):

        learner_state = []

        if self.agent_state["at"] == "lab":
            learner_state.append(0)
        else:
            learner_state.append(1)
        
        facing_indecies = dict(zip(self.object_list, np.arange(len(self.object_list))))
        facing_index = facing_indecies[self.agent_state["facing"][0]]
        facing_state = [0] * len(self.object_list)
        facing_state[facing_index] = 1

        learner_state = learner_state + facing_state

        # Add distances and orientation to all static objects, add true false for 
        odom_pose_with_cov_stamped = rospy.wait_for_message("amcl_pose", PoseWithCovarianceStamped, rospy.Duration(1))
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
            q_0 = [odom_pose.orientation.x, odom_pose.orientation.y, odom_pose.orientation.z, odom_pose.orientation.w]
            rel_orientation = quaternion_multiply(q_0, q_1_inverse)
            rot.append(rel_orientation[2])
            rot.append(rel_orientation[3])

            # Add dxdy and rot to learner state
            learner_state = learner_state + dxdy + rot

            odom_pose_stamped = self.pose_with_covariance_stamed_to_pose_stamped(odom_pose_with_cov_stamped)
            waypoint_pose_stamped = self.waypoint_to_pose_stamped(pose)

            plan = self.make_plan_client(odom_pose_stamped, waypoint_pose_stamped, 0.25)
            if len(plan.plan.poses) > 0:
                learner_state.append(1)
            else:
                learner_state.append(0)

        learner_state.append(self.bumper_pressed)

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
    Manager()