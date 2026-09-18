from pathlib import Path
from ..common import utilities as util



import os
import numpy as np
import torch
import torch.nn.functional as F
from statistics import mean
from . import sac_utils as utils
from .sac_critic import DoubleQCritic as critic_model
from .sac_actor import DiagGaussianActor as actor_model


from turtlebot3_drl.common import utilities as util


from torch.utils.tensorboard import SummaryWriter

from ..common.settings import BATCH_SIZE, BUFFER_SIZE, DISCOUNT_FACTOR, LEARNING_RATE, TAU, STEP_TIME



class SACAgent(object):
    """SAC algorithm."""

    def __init__(
        self,
        state_dim, 
        action_dim,
        device,
        max_action,
        buffer_size=1000000,
        discount=0.99,
        init_temperature=0.1,
        alpha_lr=1e-4,
        alpha_betas=(0.9, 0.999),
        actor_lr=1e-4,
        actor_betas=(0.9, 0.999),
        actor_update_frequency=1,
        critic_lr=1e-4,
        critic_betas=(0.9, 0.999),
        critic_tau=0.005,
        critic_target_update_frequency=2,
        learnable_temperature=True,
        save_every=0,
        load_model=False,
        log_dist_and_hist = False,
        save_directory=Path("src/drl_navigation_ros2/models/SAC"),
        model_name="SAC",
        load_directory=Path("src/drl_navigation_ros2/models/SAC"),

  

    ):
        super().__init__()

        self.state_dim = state_dim
        self.action_dim = action_dim
        self.action_range = (-max_action, max_action)
        self.device = torch.device(device)
        self.discount = discount
        self.critic_tau = critic_tau
        self.actor_update_frequency = actor_update_frequency
        self.critic_target_update_frequency = critic_target_update_frequency
        self.learnable_temperature = learnable_temperature
        self.save_every = save_every
        self.model_name = model_name
        self.save_directory = save_directory
        self.log_dist_and_hist = log_dist_and_hist
        self.batch_size = BATCH_SIZE
        self.buffer_size = BUFFER_SIZE
        self.discount = DISCOUNT_FACTOR
        self.tau = TAU
        self.learning_rate = LEARNING_RATE
        self.step_time = STEP_TIME



        self.train_metrics_dict = { "train_critic/loss_av": [],
                                    "train_actor/loss_av": [],
                                    "train_actor/target_entropy_av": [],
                                    "train_actor/entropy_av": [],
                                    "train_alpha/loss_av": [],
                                    "train_alpha/value_av": [],
                                    "train/batch_reward_av": []
        }
        # Fix : Prevent SummaryWriter from being pickled
        def __getstate__(self):
            state = self.__dict__.copy()
            # Remove unpicklable objects
            if 'writer' in state:
                del state['writer']
            return state

        def __setstate__(self, state):
            self.__dict__.update(state)
            # Reinitialize unpicklable objects
            self.writer = SummaryWriter()
        # Fix 
        self.critic = critic_model(
            # obs_dim=self.state_dim,
            obs_dim= 44,
            action_dim=action_dim,
            hidden_dim=1024,
            hidden_depth=2,
        ).to(self.device)
        self.critic_target = critic_model(
            # obs_dim=self.state_dim,
            obs_dim= 44,
            action_dim=action_dim,
            hidden_dim=1024,
            hidden_depth=2,
        ).to(self.device)
        self.critic_target.load_state_dict(self.critic.state_dict())

        self.actor = actor_model(
            # obs_dim=self.state_dim,
            obs_dim= 44,
            action_dim=action_dim,
            hidden_dim=1024,
            hidden_depth=2,
            log_std_bounds=[-5, 2],
        ).to(self.device)

        if load_model:
            self.load(filename=model_name, directory=load_directory)

        self.log_alpha = torch.tensor(np.log(init_temperature)).to(self.device)
        self.log_alpha.requires_grad = True
        # set target entropy to -|A|
        self.target_entropy = -action_dim

        # optimizers
        self.actor_optimizer = torch.optim.Adam(
            self.actor.parameters(), lr=actor_lr, betas=actor_betas
        )

        self.critic_optimizer = torch.optim.Adam(
            self.critic.parameters(), lr=critic_lr, betas=critic_betas
        )

        self.log_alpha_optimizer = torch.optim.Adam(
            [self.log_alpha], lr=alpha_lr, betas=alpha_betas
        )

        self.critic_target.train()

        self.actor.train(True)
        self.critic.train(True)
        self.step = 0
        self.writer = SummaryWriter()

        ##fix
        self.networks = {
            "actor": self.actor,
            "critic": self.critic,
            "critic_target": self.critic_target}
        ##

    def save(self, filename, directory):
        torch.save(self.actor.state_dict(), "%s/%s_actor.pth" % (directory, filename))
        torch.save(self.critic.state_dict(), "%s/%s_critic.pth" % (directory, filename))
        torch.save(
            self.critic_target.state_dict(),
            "%s/%s_critic_target.pth" % (directory, filename),
        )

    # def load(self, filename, directory):
    #     self.actor.load_state_dict(
    #         torch.load("%s/%s_actor.pth" % (directory, filename))
    #     )
    #     self.critic.load_state_dict(
    #         torch.load("%s/%s_critic.pth" % (directory, filename))
    #     )
    #     self.critic_target.load_state_dict(
    #         torch.load("%s/%s_critic_target.pth" % (directory, filename))
    #     )
    #     print(f"Loaded weights from: {directory}")


    def load_session_model(self, session_name, episode=0):
        """
        Load a saved model for testing or continued training.
        
        Args:
            session_name: Name of the session directory (e.g. "sac_10")
            episode: Episode number to load (default: 0 for latest)
        """
        # Construct the full session directory path
        session_dir = os.path.join(self.sm.session_root, session_name)
        
        if not os.path.exists(session_dir):
            raise FileNotFoundError(f"Session directory not found: {session_dir}")

        print(f"[INFO] Loading session from: {session_dir}")
        print(f"[INFO] Stage: {self.sm.stage}, Episode: {episode}")

        # Try to find the latest episode if not specified
        if episode == 0:
            # Find all actor files and get the latest episode
            actor_files = [f for f in os.listdir(session_dir) if f.startswith(f'actor_stage{self.sm.stage}_')]
            if not actor_files:
                raise FileNotFoundError(f"No model weights found in {session_dir}")
            
            # Extract episode numbers
            episode_numbers = [int(f.split('_episode')[-1].split('.')[0]) for f in actor_files]
            episode = max(episode_numbers)
            print(f"[INFO] Loading latest episode: {episode}")

        # Construct file paths
        actor_path = os.path.join(session_dir, f'actor_stage{self.sm.stage}_episode{episode}.pt')
        critic_path = os.path.join(session_dir, f'critic_stage{self.sm.stage}_episode{episode}.pt')
        critic_target_path = os.path.join(session_dir, f'critic_target_stage{self.sm.stage}_episode{episode}.pt')

        # Load weights
        try:
            self.actor.load_state_dict(torch.load(actor_path))
            print(f"[INFO] Loaded actor weights from: {actor_path}")
            
            self.critic.load_state_dict(torch.load(critic_path))
            print(f"[INFO] Loaded critic weights from: {critic_path}")
            
            if hasattr(self, 'critic_target'):
                self.critic_target.load_state_dict(torch.load(critic_target_path))
                print(f"[INFO] Loaded critic_target weights from: {critic_target_path}")
        except Exception as e:
            raise RuntimeError(f"Error loading model weights: {str(e)}")

        # Load replay buffer if in training mode
        if self.training:
            buffer_path = os.path.join(session_dir, f'stage{self.sm.stage}_latest_buffer.pkl')
            if os.path.exists(buffer_path):
                try:
                    self.replay_buffer.buffer = self.sm.load_replay_buffer(self.buffer_size, buffer_path)
                    print(f"[INFO] Loaded replay buffer from: {buffer_path}")
                except Exception as e:
                    print(f"[WARNING] Failed to load replay buffer: {str(e)}")

        # Load training graphs
        try:
            self.total_steps = self.graph.set_graphdata(self.sm.load_graphdata(), episode)
            print(f"[INFO] Global steps: {self.total_steps}")
        except Exception as e:
            print(f"[WARNING] Failed to load graph data: {str(e)}")

        print(f"[INFO] Successfully loaded model from {session_name} (episode {episode})")
    # def _train(self, replay_buffer, iterations=1, batch_size=128):
    #     for _ in range(iterations):
    #         self.update(
    #             replay_buffer=replay_buffer, step=self.step, batch_size=batch_size
    #         )

    #     for key, value in self.train_metrics_dict.items():
    #         if len(value):
    #             self.writer.add_scalar(key, mean(value), self.step)
    #         self.train_metrics_dict[key] = []
    #     self.step += 1

    #     if self.save_every > 0 and self.step % self.save_every == 0:
    #         self.save(filename=self.model_name, directory=self.save_directory)
    def _train(self, replay_buffer, iterations=1, batch_size=128):
        all_loss_c = []
        all_loss_a = []
        for _ in range(iterations):
            loss_c, loss_a = self.update(
                replay_buffer=replay_buffer, step=self.step, batch_size=batch_size
            )
            all_loss_c.append(loss_c)
            all_loss_a.append(loss_a)

        for key, value in self.train_metrics_dict.items():
            if len(value):
                self.writer.add_scalar(key, mean(value), self.step)
            self.train_metrics_dict[key] = []
        self.step += 1

        if self.save_every > 0 and self.step % self.save_every == 0:
            self.save(filename=self.model_name, directory=self.save_directory)

        return mean(all_loss_c), mean(all_loss_a)

    @property
    def alpha(self):
        return self.log_alpha.exp()
    
    def get_action(self, state, is_training, step=None, visualize=False):
        state = torch.FloatTensor(state).to(self.device)
        dist = self.actor(state, visualize)  # returns SquashedNormal

        action = dist.sample()

        if is_training:
            noise = torch.normal(mean=0, std=0.2, size=action.shape).to(self.device)
            action = action + noise

        action = torch.clamp(action, self.action_range[0], self.action_range[1])
        return action.cpu().detach().numpy().tolist()


    def get_action_random(self):
        return [np.clip(np.random.uniform(-1.0, 1.0), -1.0, 1.0) for _ in range(self.action_dim)]



   # def get_action(self, obs, add_noise):
    #    if add_noise:
     #       return (
      #          self.act(obs) + np.random.normal(0, 0.2, size=self.action_dim)
       #     ).clip(self.action_range[0], self.action_range[1])
        #else:
         #   return self.act(obs)

    #def get_action_random(self):
     #   return [np.clip(np.random.uniform(-1.0, 1.0), -1.0, 1.0) for _ in range(self.action_dim)]
   
        




    def act(self, obs, sample=False):
        obs = torch.FloatTensor(obs).to(self.device)
        obs = obs.unsqueeze(0)
        dist = self.actor(obs)
        action = dist.sample() if sample else dist.mean
        action = action.clamp(*self.action_range)
        assert action.ndim == 2 and action.shape[0] == 1
        return utils.to_np(action[0])

    # def update_critic(self, obs, action, reward, next_obs, done, step):
    #     dist = self.actor(next_obs)
    #     next_action = dist.rsample()
    #     log_prob = dist.log_prob(next_action).sum(-1, keepdim=True)
    #     target_Q1, target_Q2 = self.critic_target(next_obs, next_action)
    #     target_V = torch.min(target_Q1, target_Q2) - self.alpha.detach() * log_prob
    #     target_Q = reward + ((1 - done) * self.discount * target_V)
    #     target_Q = target_Q.detach()

    #     # get current Q estimates
    #     current_Q1, current_Q2 = self.critic(obs, action)
    #     critic_loss = F.mse_loss(current_Q1, target_Q) + F.mse_loss(
    #         current_Q2, target_Q
    #     )
    #     self.train_metrics_dict["train_critic/loss_av"].append(critic_loss.item())
    #     self.writer.add_scalar("train_critic/loss", critic_loss, step)

    #     # Optimize the critic
    #     self.critic_optimizer.zero_grad()
    #     critic_loss.backward()
    #     self.critic_optimizer.step()
    #     if self.log_dist_and_hist:
    #         self.critic.log(self.writer, step)
    #     # Fix
    #     loss_critic = self.update_critic(obs, action, reward, next_obs, done, step)
    #     loss_actor = self.update_actor(obs, step)

    #     # Add this line to return both losses
    #     return loss_critic, loss_actor
    #     # Fix 

    def update_critic(self, obs, action, reward, next_obs, done, step):
        dist = self.actor(next_obs)
        next_action = dist.rsample()
        log_prob = dist.log_prob(next_action).sum(-1, keepdim=True)
        target_Q1, target_Q2 = self.critic_target(next_obs, next_action)
        target_V = torch.min(target_Q1, target_Q2) - self.alpha.detach() * log_prob
        target_Q = reward + ((1 - done) * self.discount * target_V)
        target_Q = target_Q.detach()

        # get current Q estimates
        current_Q1, current_Q2 = self.critic(obs, action)
        critic_loss = F.mse_loss(current_Q1, target_Q) + F.mse_loss(current_Q2, target_Q)
        
        self.train_metrics_dict["train_critic/loss_av"].append(critic_loss.item())
        self.writer.add_scalar("train_critic/loss", critic_loss, step)

        # Optimize the critic
        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()

        if self.log_dist_and_hist:
            self.critic.log(self.writer, step)

        # Compute actor loss here instead of recursively calling update_critic again
        loss_actor = self.update_actor_and_alpha(obs, step)

        # Return both losses
        return critic_loss.item(), loss_actor

    def update_actor_and_alpha(self, obs, step):
        dist = self.actor(obs)
        action = dist.rsample()
        log_prob = dist.log_prob(action).sum(-1, keepdim=True)
        actor_Q1, actor_Q2 = self.critic(obs, action)

        actor_Q = torch.min(actor_Q1, actor_Q2)
        actor_loss = (self.alpha.detach() * log_prob - actor_Q).mean()
        self.train_metrics_dict["train_actor/loss_av"].append(actor_loss.item())
        self.train_metrics_dict["train_actor/target_entropy_av"].append(self.target_entropy)
        self.train_metrics_dict["train_actor/entropy_av"].append(-log_prob.mean().item())
        self.writer.add_scalar("train_actor/loss", actor_loss, step)
        self.writer.add_scalar("train_actor/target_entropy", self.target_entropy, step)
        self.writer.add_scalar("train_actor/entropy", -log_prob.mean(), step)

        # optimize the actor
        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()
        if self.log_dist_and_hist:
            self.actor.log(self.writer, step)

        if self.learnable_temperature:
            self.log_alpha_optimizer.zero_grad()
            alpha_loss = (
                self.alpha * (-log_prob - self.target_entropy).detach()
            ).mean()
            self.train_metrics_dict["train_alpha/loss_av"].append(alpha_loss.item())
            self.train_metrics_dict["train_alpha/value_av"].append(self.alpha.item())
            self.writer.add_scalar("train_alpha/loss", alpha_loss, step)
            self.writer.add_scalar("train_alpha/value", self.alpha, step)
            alpha_loss.backward()
            self.log_alpha_optimizer.step()
        return actor_loss.item()


    # def update(self, replay_buffer, step, batch_size):
    #     (
    #         batch_states,
    #         batch_actions,
    #         batch_rewards,
    #         batch_dones,
    #         batch_next_states,
    #     ) = replay_buffer.sample_batch(batch_size)
    def update(self, replay_buffer, step, batch_size):
        (
            batch_states,
            batch_actions,
            batch_rewards,
            batch_next_states,
            batch_dones,
        ) = replay_buffer.sample(batch_size)

        state = torch.Tensor(batch_states).to(self.device)
        next_state = torch.Tensor(batch_next_states).to(self.device)
        action = torch.Tensor(batch_actions).to(self.device)
        reward = torch.Tensor(batch_rewards).to(self.device)
        done = torch.Tensor(batch_dones).to(self.device)
        self.train_metrics_dict["train/batch_reward_av"].append(batch_rewards.mean().item())
        self.writer.add_scalar("train/batch_reward", batch_rewards.mean(), step)

        loss_critic, loss_actor = self.update_critic(state, action, reward, next_state, done, step)

        if step % self.actor_update_frequency == 0:
            self.update_actor_and_alpha(state, step)  # You could also refactor this to return the loss

        if step % self.critic_target_update_frequency == 0:
            utils.soft_update_params(self.critic, self.critic_target, self.critic_tau)

        return loss_critic, loss_actor

 

    def prepare_state(self, latest_scan, distance, cos, sin, collision, goal, action):
        # update the returned data from ROS into a form used for learning in the current model
        latest_scan = np.array(latest_scan)

        inf_mask = np.isinf(latest_scan)
        latest_scan[inf_mask] = 7.0

        max_bins = self.state_dim - 5
        bin_size = int(np.ceil(len(latest_scan) / max_bins))

        # Initialize the list to store the minimum values of each bin
        min_values = []

        # Loop through the data and create bins
        for i in range(0, len(latest_scan), bin_size):
            # Get the current bin
            bin = latest_scan[i : i + min(bin_size, len(latest_scan) - i)]
            # Find the minimum value in the current bin and append it to the min_values list
            min_values.append(min(bin))
        state = min_values + [distance, cos, sin] + [action[0], action[1]]

        assert len(state) == self.state_dim
        terminal = 1 if collision or goal else 0

        return state, terminal
    def get_model_parameters(self):
        return {
            "state_dim": self.state_dim,
            "action_dim": self.action_dim,
            "buffer_size": self.buffer_size,
            "discount": self.discount,
            "init_temperature": float(self.log_alpha.exp().item()),
            "actor_lr": self.actor_optimizer.param_groups[0]["lr"],
            "critic_lr": self.critic_optimizer.param_groups[0]["lr"],
            "alpha_lr": self.log_alpha_optimizer.param_groups[0]["lr"],
            "actor_update_frequency": self.actor_update_frequency,
            "critic_target_update_frequency": self.critic_target_update_frequency,
            "learnable_temperature": self.learnable_temperature,
        }
    def get_model_configuration(self):
        return {
            "actor_network": str(self.actor),
            "critic_network": str(self.critic),
            "actor_optimizer": str(self.actor_optimizer),
            "critic_optimizer": str(self.critic_optimizer),
            "temperature_optimizer": str(self.log_alpha_optimizer),
            "learnable_temperature": self.learnable_temperature,
            "log_alpha": self.log_alpha.item(),
            "target_entropy": self.target_entropy,
        }


