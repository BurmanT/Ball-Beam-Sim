import os
import glob
import time
from datetime import datetime

import torch
import numpy as np

from agent.PPO import PPO
import gymnasium as gym

################################### Training ###################################
def train():
    print("============================================================================================")

    ####### initialize environment hyperparameters ######
    env_name = "MiniGrid-NineRoomsKeyGoal-v0"

    max_ep_len = 500                   # max timesteps in one episode
    max_training_timesteps = int(1e9)   # break training loop if timeteps > max_training_timesteps

    print_freq = max_ep_len * 10        # print avg reward in the interval (in num timesteps)
    log_freq = max_ep_len * 2           # log avg reward in the interval (in num timesteps)
    save_model_freq = int(1e5)          # save model frequency (in num timesteps)

    ## Note : print/log frequencies should be > than max_ep_len

    ################ PPO hyperparameters ################
    update_timestep = max_ep_len*6      # update policy every n timesteps
    K_epochs = 15               # update policy for K epochs in one PPO update

    eps_clip = 0.2          # clip parameter for PPO
    gamma = 0.99            # discount factor

    lr_actor = 0.0004       # learning rate for actor network
    lr_critic = 0.001       # learning rate for critic network

    random_seed = 0         # set random seed if required (0 = no random seed)
    #####################################################

    print("training environment name : " + env_name)

    # env = gym.make(env_name, render_mode = "human")
    env = gym.make(env_name, max_steps = max_ep_len)
    # env = RGBImgObsWrapper(env)

    # state space dimension
    state_dim = env.observation_space.shape

    # action space dimension

    action_dim = env.action_space.n-1


    ################### checkpointing ###################
    run_num_pretrained = 0      #### change this to prevent overwriting weights in same env_name folder

    directory = "PPO_preTrained"
    if not os.path.exists(directory):
          os.makedirs(directory)

    directory = directory + '/' + env_name + '/'
    if not os.path.exists(directory):
          os.makedirs(directory)


    checkpoint_path = directory + "PPO_{}_{}_{}.pth".format(env_name, random_seed, run_num_pretrained)
    print("save checkpoint path : " + checkpoint_path)
    #####################################################


    ############# print all hyperparameters #############
    print("--------------------------------------------------------------------------------------------")
    print("max training timesteps : ", max_training_timesteps)
    print("max timesteps per episode : ", max_ep_len)
    print("model saving frequency : " + str(save_model_freq) + " timesteps")
    print("log frequency : " + str(log_freq) + " timesteps")
    print("printing average reward over episodes in last : " + str(print_freq) + " timesteps")
    print("--------------------------------------------------------------------------------------------")
    print("state space dimension : ", state_dim)
    print("action space dimension : ", action_dim)
    print("--------------------------------------------------------------------------------------------")
    print("Initializing a discrete action space policy")
    print("--------------------------------------------------------------------------------------------")
    print("PPO update frequency : " + str(update_timestep) + " timesteps")
    print("PPO K epochs : ", K_epochs)
    print("PPO epsilon clip : ", eps_clip)
    print("discount factor (gamma) : ", gamma)
    print("--------------------------------------------------------------------------------------------")
    print("optimizer learning rate actor : ", lr_actor)
    print("optimizer learning rate critic : ", lr_critic)
    if random_seed:
        print("--------------------------------------------------------------------------------------------")
        print("setting random seed to ", random_seed)
        torch.manual_seed(random_seed)
        env.seed(random_seed)
        np.random.seed(random_seed)
    #####################################################

    print("============================================================================================")

    ################# training procedure ################

    # initialize a PPO agent
    ppo_agent = PPO(state_dim, action_dim, lr_actor, lr_critic, gamma, K_epochs, eps_clip)

    # track total training time
    start_time = datetime.now().replace(microsecond=0)
    print("Started training at (GMT) : ", start_time)

    print("============================================================================================")

    # printing and logging variables
    print_running_reward = 0
    print_running_episodes = 0

    log_running_reward = 0
    log_running_episodes = 0

    time_step = 0
    i_episode = 0

    done_arr = []
    reward_arr = []
    # training loop
    while time_step <= max_training_timesteps:

        state, info = env.reset()
        current_ep_reward = 0
        print_avg_reward = 0

        for t in range(1, max_ep_len+1):

            # select action with policy
            state = np.expand_dims(state, axis=0)
            action = ppo_agent.select_action(state)   
            state, reward, terminated, truncated, info = env.step(action)

            # saving reward and is_terminals
            ppo_agent.buffer.rewards.append(reward)
            if terminated or truncated:
                ppo_agent.buffer.is_terminals.append(True)
            else:
                ppo_agent.buffer.is_terminals.append(False)
            # if truncated:
            #     ppo_agent.buffer.is_terminals.append(0)

            time_step +=1
            current_ep_reward += reward

            # update PPO agent
            if time_step % update_timestep == 0:
                ppo_agent.update()

            # printing average reward
            if time_step % print_freq == 0:

                # print average reward till last episode
                print_avg_reward = print_running_reward / print_running_episodes
                print_avg_reward = round(print_avg_reward, 2)

                print("Episode : {} \t\t Timestep : {} \t\t Average Reward : {}".format(i_episode, time_step, print_avg_reward))

                print_running_reward = 0
                print_running_episodes = 0

                if print_avg_reward > 0.92:
                    break

            # save model weights
            if time_step % save_model_freq == 0:
                print("--------------------------------------------------------------------------------------------")
                print("saving model at : " + checkpoint_path)
                ppo_agent.save(checkpoint_path)
                print("model saved")
                print("Elapsed Time  : ", datetime.now().replace(microsecond=0) - start_time)
                print("--------------------------------------------------------------------------------------------")

            # break; if the episode is over
            if terminated:
                done_arr.append(1)
                reward_arr.append(current_ep_reward)
                break
            if truncated:
                done_arr.append(0)
                reward_arr.append(current_ep_reward)

        if print_avg_reward > 0.92:
            print(print_avg_reward)
            print("saving converged model at : " + checkpoint_path)
            ppo_agent.save(checkpoint_path)
            print("model converged")            
            break

        print_running_reward += current_ep_reward
        print_running_episodes += 1

        log_running_reward += current_ep_reward
        log_running_episodes += 1

        i_episode += 1
        # if len(done_arr) > 50 and np.mean(done_arr[-10:])>0.9:
        #     print("saving converged model at : " + checkpoint_path)
        #     ppo_agent.save(checkpoint_path)
        #     print("model converged")            
        #     break
    env.close()

    # print total training time
    print("============================================================================================")
    end_time = datetime.now().replace(microsecond=0)
    print("Started training at (GMT) : ", start_time)
    print("Finished training at (GMT) : ", end_time)
    print("Total training time  : ", end_time - start_time)
    print("============================================================================================")


if __name__ == '__main__':

    train()