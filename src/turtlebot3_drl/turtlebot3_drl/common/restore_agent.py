import os
import torch
import pickle

from turtlebot3_drl.drl_agent import SACAgent


# ------------------ CONFIGURATION ------------------

stage = 4
episode = 700
session_name = "sac_10"
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

machine_name = os.uname().nodename
base_model_dir = os.path.join(os.path.expanduser("~"), "turtlebot3_drlnav", "src", "turtlebot3_drl", "model", machine_name, session_name)

actor_path = os.path.join(base_model_dir, f"actor_stage{stage}_episode{episode}.pt")
critic_path = os.path.join(base_model_dir, f"critic_stage{stage}_episode{episode}.pt")
critic_target_path = os.path.join(base_model_dir, f"critic_target_stage{stage}_episode{episode}.pt")
output_model_path = os.path.join(base_model_dir, f"stage{stage}_agent.pkl")

# ------------------ LOAD WEIGHTS ------------------

# Instantiate a fresh agent
agent = SACAgent()  # You must make sure this builds the same network architecture

# Load weights
agent.actor.load_state_dict(torch.load(actor_path, map_location=device))
agent.critic.load_state_dict(torch.load(critic_path, map_location=device))
if hasattr(agent, "critic_target"):
    agent.critic_target.load_state_dict(torch.load(critic_target_path, map_location=device))

# Set device
agent.device = device

# ------------------ SAVE PICKLED AGENT ------------------

with open(output_model_path, "wb") as f:
    pickle.dump(agent, f, pickle.HIGHEST_PROTOCOL)

print(f"✅ Saved model to: {output_model_path}")
