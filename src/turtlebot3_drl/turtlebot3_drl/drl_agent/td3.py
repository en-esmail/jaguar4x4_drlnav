import numpy as np
import copy

import torch
import torch.nn.functional as F
import torch.nn as nn

from ..common.ounoise import OUNoise
from ..common.settings import POLICY_NOISE, POLICY_NOISE_CLIP, POLICY_UPDATE_FREQUENCY

from .off_policy_agent import OffPolicyAgent, Network

LINEAR = 0
ANGULAR = 1


class Actor(Network):
    def __init__(self, name, state_size, action_size, hidden_size):
        super(Actor, self).__init__(name)
        self.fa1 = nn.Linear(state_size, hidden_size)
        self.fa2 = nn.Linear(hidden_size, hidden_size)
        self.fa3 = nn.Linear(hidden_size, action_size)
        self.apply(super().init_weights)

    def forward(self, states, visualize=False):
        x1 = torch.relu(self.fa1(states))
        x2 = torch.relu(self.fa2(x1))
        action = torch.tanh(self.fa3(x2))
        if visualize and self.visual:
            self.visual.update_layers(states, action, [x1, x2], [self.fa1.bias, self.fa2.bias])
        return action


class Critic(Network):
    def __init__(self, name, state_size, action_size, hidden_size):
        super(Critic, self).__init__(name)
        # Q1
        self.l1 = nn.Linear(state_size, int(hidden_size / 2))
        self.l2 = nn.Linear(action_size, int(hidden_size / 2))
        self.l3 = nn.Linear(hidden_size, hidden_size)
        self.l4 = nn.Linear(hidden_size, 1)
        # Q2
        self.l5 = nn.Linear(state_size, int(hidden_size / 2))
        self.l6 = nn.Linear(action_size, int(hidden_size / 2))
        self.l7 = nn.Linear(hidden_size, hidden_size)
        self.l8 = nn.Linear(hidden_size, 1)
        self.apply(super().init_weights)

    def forward(self, states, actions):
        xs = torch.relu(self.l1(states))
        xa = torch.relu(self.l2(actions))
        x = torch.cat((xs, xa), dim=1)
        x = torch.relu(self.l3(x))
        x1 = self.l4(x)

        xs = torch.relu(self.l5(states))
        xa = torch.relu(self.l6(actions))
        x = torch.cat((xs, xa), dim=1)
        x = torch.relu(self.l7(x))
        x2 = self.l8(x)

        return x1, x2

    def Q1_forward(self, states, actions):
        xs = torch.relu(self.l1(states))
        xa = torch.relu(self.l2(actions))
        x = torch.cat((xs, xa), dim=1)
        x = torch.relu(self.l3(x))
        x1 = self.l4(x)
        return x1


class TD3(OffPolicyAgent):
    def __init__(self, device, sim_speed):
        super().__init__(device, sim_speed)

        # Placeholder epsilon to avoid AttributeError in OffPolicyAgent._train
        # (kept here for clarity — OffPolicyAgent will be made robust as well)
        self.epsilon = 0.0
        self.epsilon_minimum = 0.0

        # noise + hyperparams
        self.noise = OUNoise(action_space=self.action_size, max_sigma=0.1, min_sigma=0.1, decay_period=8000000)
        self.tau = 0.005  # Soft update rate
        self.policy_noise = POLICY_NOISE
        self.noise_clip = POLICY_NOISE_CLIP
        self.policy_freq = POLICY_UPDATE_FREQUENCY
        self.last_actor_loss = 0

        # --- Initialize actor networks ---
        self.actor = self.create_network(Actor, 'actor')
        self.actor_target = copy.deepcopy(self.actor)
        self.actor_target.to(self.device)
        self.actor_target.eval()
        self.actor_optimizer = self.create_optimizer(self.actor)

        # --- Initialize critic networks ---
        self.critic = self.create_network(Critic, 'critic')
        self.critic_target = copy.deepcopy(self.critic)
        self.critic_target.to(self.device)
        self.critic_target.eval()
        self.critic_optimizer = self.create_optimizer(self.critic)

        # Hard update to sync target networks
        self.hard_update(self.actor_target, self.actor)
        self.hard_update(self.critic_target, self.critic)

    def _soft_update(self, target, source, tau):
        with torch.no_grad():
            for target_param, source_param in zip(target.parameters(), source.parameters()):
                target_param.data.copy_(tau * source_param.data + (1.0 - tau) * target_param.data)

    def _ensure_targets(self):
        if not hasattr(self, 'actor_target') or self.actor_target is None:
            self.actor_target = copy.deepcopy(self.actor)
            self.actor_target.to(self.device)
            self.actor_target.eval()
        if not hasattr(self, 'critic_target') or self.critic_target is None:
            self.critic_target = copy.deepcopy(self.critic)
            self.critic_target.to(self.device)
            self.critic_target.eval()

    def get_action(self, state, is_training, step, visualize=False):
        state = torch.from_numpy(np.asarray(state, np.float32)).to(self.device)
        action = self.actor(state, visualize)

        if is_training:
            if not hasattr(self, 'noise') or self.noise is None:
                self.noise = OUNoise(action_space=self.action_size, max_sigma=0.1, min_sigma=0.1, decay_period=8000000)
            noise_np = copy.deepcopy(self.noise.get_noise(step))
            noise = torch.from_numpy(noise_np).to(self.device)
            action = torch.clamp(action + noise, -1.0, 1.0)

        return action.detach().cpu().numpy().tolist()

    def get_action_random(self):
        return [np.clip(np.random.uniform(-1.0, 1.0), -1.0, 1.0)] * self.action_size

    def train(self, state, action, reward, state_next, done):
        self._ensure_targets()

        state = state.to(self.device)
        action = action.to(self.device)
        reward = reward.to(self.device)
        state_next = state_next.to(self.device)
        done = done.to(self.device)

        noise = (torch.randn_like(action) * self.policy_noise).clamp(-self.noise_clip, self.noise_clip)
        action_next = (self.actor_target(state_next) + noise).clamp(-1.0, 1.0)

        Q1_next, Q2_next = self.critic_target(state_next, action_next)
        Q_next = torch.min(Q1_next, Q2_next)
        Q_target = reward + (1 - done) * self.discount_factor * Q_next

        Q1, Q2 = self.critic(state, action)
        loss_critic = self.loss_function(Q1, Q_target) + self.loss_function(Q2, Q_target)
        self.critic_optimizer.zero_grad()
        loss_critic.backward()
        nn.utils.clip_grad_norm_(self.critic.parameters(), max_norm=2.0, norm_type=2)
        self.critic_optimizer.step()

        if hasattr(self, 'iteration') and (self.iteration % self.policy_freq == 0):
            loss_actor = -1 * self.critic.Q1_forward(state, self.actor(state)).mean()
            self.actor_optimizer.zero_grad()
            loss_actor.backward()
            nn.utils.clip_grad_norm_(self.actor.parameters(), max_norm=2.0, norm_type=2)
            self.actor_optimizer.step()

            self._soft_update(self.actor_target, self.actor, self.tau)
            self._soft_update(self.critic_target, self.critic, self.tau)
            self.last_actor_loss = loss_actor.detach().cpu()

        return [loss_critic.detach().cpu(), self.last_actor_loss]

