import json
import os
import numpy as np
from random import random
import params

from manager import Manager
from regular_policy_gradient import RegularPolicyGradient

class Learner:
    def __init__(self, failed_operator_name, init_obs, init_actions, episode_num, load_model_flag, log_dir, trial_number):
        
        seed = np.random.randint(0, 100)
        obs_size = init_obs.shape[0]
        action_size = len(init_actions)
        
        self.timestep = 0
        self.episode = 0
        self.failed_operator_name = failed_operator_name

        self.agent = RegularPolicyGradient(num_actions=action_size,\
                                            input_size=obs_size, hidden_layer_size=params.NUM_HIDDEN,
                                            learning_rate=params.LEARNING_RATE, gamma=params.GAMMA, decay_rate=params.DECAY_RATE,
                                            greedy_e_epsilon=params.MAX_EPSILON, actions_id=init_actions, 
                                            random_seed=seed, load_model_flag=load_model_flag, episode_num=episode_num, log_dir=log_dir, failed_operator_name=failed_operator_name, trial_number=trial_number)


    def get_action(self, obs, done=False, action=None):
        '''
        This function returns action for given observation
        '''
        while True:
            action = self.agent.process_step(x = obs, exploring = True, action = action)
            self.timestep += 1
            
            return action

    def check_convergence(self):
        # this checks if we have to stop learning and save a policy
        if np.mean(self.R[-params.NO_OF_EPS_TO_CHECK:]) > params.SCORE_TO_CHECK and params.np.mean(self.R[-10:]) > params.SCORE_TO_CHECK: # check the average reward for last 70 episodes
                                # for future we can write an evaluation function here which runs a evaluation on the current policy.
                                if  np.sum(self.dones[-params.NO_OF_DONES_TO_CHECK:]) > params.NO_OF_SUCCESSFUL_DONE and np.mean(self.dones[-10:]) > params.NO_OF_SUCCESSFUL_DONE/params.NO_OF_DONES_TO_CHECK: # and check the success percentage of the agent > 80%.
                                    if abs(np.mean(self.dones[-params.NO_OF_DONES_TO_CHECK:]) - np.mean(self.dones[-10:])) < 0.05:
                                        print ("The agent has learned to reach the subgoal")
                                        return True
        else:
            return False