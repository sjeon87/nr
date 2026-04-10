#!/usr/bin/env python3
# Copyright (c) 2026 University of Moratuwa
# Author: Nipuna Dulara
# Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
#
# SPDX-License-Identifier: GPL-2.0-only
"""
PyTorch PPO agent for the NR AI scheduler (msg-interface).
This script reads per-flow observations from and writes scheduling weights
to Boost.Interprocess shared memory using the pybind11 bindings exposed by
ns3ai_nr_sched_py.  It implements the Proximal Policy Optimization (PPO)
algorithm with an Actor Critic architecture.
The agent is designed to run in a separate process alongside the C++
simulation (nr-ai-sched-msg.cc).  The C++ side creates the shared memory
segment; this script attaches to it.
Architecture:
    Actor:  Linear(num_features, 256) -> ReLU -> Linear(256, 256) -> ReLU
            -> Linear(256, num_flows) -> Softplus
    Critic: Linear(num_features * num_flows, 256) -> ReLU -> Linear(256, 1)
Key differences from the GSoC 2024 PPO agent (rl-sched-gym-env-ppo.py):
    1. Shared memory IPC instead of ZMQ sockets
    2. 9 input features per flow instead of 4
    3. 256 hidden units instead of 64
    4. Softplus activation instead of Tanh + rescaling
    5. GAE advantage estimation instead of simple discounted returns
"""
import argparse
import csv
import os
import sys
import time
from collections import deque
from pathlib import Path
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions import Normal
# Import the pybind11 bindings for shared memory access
# The ns3ai_nr_sched_py module is built by CMake alongside the C++
# example.  It provides access to NrSchedEnvMsg, NrSchedActMsg,
# and the Ns3AiMsgInterfaceImpl synchronisation class.
try:
    import ns3ai_nr_sched_py as ns3ai
except ImportError:
    print(
        "ERROR: Cannot import ns3ai_nr_sched_py.\n"
        "Make sure the pybind11 module was built:\n"
        "  ./ns3 build\n"
        "Then run from the example directory or add it to PYTHONPATH.",
        file=sys.stderr,
    )
    sys.exit(1)
# Constants
MAX_FLOWS = 64  # Must match MAX_FLOWS in nr-mac-scheduler-ai-msg-structs.h
NUM_FEATURES = 9  # Features per flow: rnti, lcId, fiveQi, priority,
#                    holDelay, cqi, bsr, avgTput, potentialTput
SHM_SEGMENT = "NrAiSchedSeg"
SHM_CPP2PY = "NrAiSchedEnv"
SHM_PY2CPP = "NrAiSchedAct"
SHM_LOCK = "NrAiSchedLock"

# Actor-Critic Network

class ActorCritic(nn.Module):
    """
    Actor-Critic neural network for PPO.
    The actor outputs per-flow scheduling weights via a Softplus
    activation, ensuring all weights are positive without manual
    rescaling. The critic estimates the state value from the flattened
    observation vector (num_flows * num_features).
    Parameters
    ----------
    num_flows : int
        Maximum number of active flows (determines output dimension).
    num_features : int
        Number of features per flow (default 9).
    hidden_dim : int
        Size of hidden layers (default 256).
    """
    def __init__(self, num_flows, num_features=NUM_FEATURES, hidden_dim=256):
        super().__init__()
        self.num_flows = num_flows
        self.num_features = num_features
        # Actor: per-flow processing -> weight output
        # Input: (batch, num_features) for each flow
        # Output: mean and log_std for Normal distribution
        self.actor_shared = nn.Sequential(
            nn.Linear(num_features, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
        )
        self.actor_mean = nn.Linear(hidden_dim, 1)
        self.actor_log_std = nn.Linear(hidden_dim, 1)
        # Critic: global state -> value estimate
        # Input: flattened (batch, num_flows * num_features)
        self.critic = nn.Sequential(
            nn.Linear(num_flows * num_features, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1),
        )
        # Softplus for positive weights
        self.softplus = nn.Softplus()
    def forward_actor(self, flow_obs):
        """
        Compute action distribution parameters for a single flow.
        Parameters
        ----------
        flow_obs : torch.Tensor
            Shape (batch, num_features) 
        Returns
        -------
        mean : torch.Tensor
            Mean of the Normal distribution (passed through Softplus).
        std : torch.Tensor
            Standard deviation of the Normal distribution.
        """
        h = self.actor_shared(flow_obs)
        mean = self.softplus(self.actor_mean(h))  # Positive weights
        log_std = torch.clamp(self.actor_log_std(h), -2.0, 2.0)
        std = torch.exp(log_std)
        return mean.squeeze(-1), std.squeeze(-1)
    def forward_critic(self, state_flat):
        """
        Compute state value estimate.
        Parameters
        ----------
        state_flat : torch.Tensor
            Shape (batch, num_flows * num_features).
        Returns
        -------
        value : torch.Tensor
            Shape (batch,).
        """
        return self.critic(state_flat).squeeze(-1)
    def act(self, state, mask):
        """
        Select actions for all flows given the current state.
        Parameters
        ----------
        state : np.ndarray
            Shape (num_flows, num_features).
        mask : list[bool]
            Which flows are active.
        Returns
        -------
        actions : np.ndarray
            Shape (num_active_flows,)- scheduling weights.
        log_probs : torch.Tensor
            Log-probabilities of the selected actions.
        value : torch.Tensor
            Critic's value estimate for this state.
        """
        state_t = torch.from_numpy(state).float()
        actions = []
        log_probs = []
        eps = 1e-6
        for i in range(self.num_flows):
            if i < len(mask) and mask[i]:
                flow_obs = state_t[i].unsqueeze(0)  # (1, num_features)
                mean, std = self.forward_actor(flow_obs)
                dist = Normal(mean, std)
                action = dist.sample()
                action = torch.clamp(action, eps, 100.0)  # Reasonable range
                actions.append(action.item())
                log_probs.append(dist.log_prob(action))
            else:
                actions.append(0.0)
                log_probs.append(torch.tensor(0.0))
        # Critic value
        state_flat = state_t.flatten().unsqueeze(0)  # (1, num_flows * num_features)
        value = self.forward_critic(state_flat)
        log_probs_t = torch.stack(log_probs)
        return np.array(actions), log_probs_t, value.squeeze()
    def evaluate(self, states, actions, masks):
        """
        Evaluate states and actions for PPO update.
        Parameters
        ----------
        states : torch.Tensor
            Shape (batch, num_flows, num_features).
        actions : torch.Tensor
            Shape (batch, num_flows).
        masks : list[list[bool]]
            Per step flow activity masks.
        Returns
        -------
        log_probs : torch.Tensor
            Shape (batch, num_flows).
        values : torch.Tensor
            Shape (batch,).
        entropy : torch.Tensor
            Scalar mean entropy.
        """
        batch_size = states.shape[0]
        all_log_probs = []
        all_entropies = []
        for i in range(self.num_flows):
            flow_obs = states[:, i, :]  # (batch, num_features)
            mean, std = self.forward_actor(flow_obs)
            dist = Normal(mean, std)
            log_prob = dist.log_prob(actions[:, i])
            entropy = dist.entropy()
            # Zero out inactive flows
            flow_mask = torch.tensor(
                [m[i] if i < len(m) else False for m in masks],
                dtype=torch.float32,
            )
            log_prob = log_prob * flow_mask
            entropy = entropy * flow_mask
            all_log_probs.append(log_prob)
            all_entropies.append(entropy)
        log_probs = torch.stack(all_log_probs, dim=1)  # (batch, num_flows)
        entropies = torch.stack(all_entropies, dim=1).mean()
        # Critic values
        states_flat = states.view(batch_size, -1)  # (batch, num_flows * num_features)
        values = self.forward_critic(states_flat)
        return log_probs, values, entropies
# Rollout Buffer
class RolloutBuffer:
    """
    Stores experience tuples for PPO updates.
    Collects (state, action, log_prob, reward, done, mask) tuples
    during rollout, then provides batched data for the PPO update step.
    Parameters
    ----------
    maxlen : int
        Maximum number of transitions to store before forcing an update.
    """
    def __init__(self, maxlen=1000):
        self.states = deque(maxlen=maxlen)
        self.actions = deque(maxlen=maxlen)
        self.log_probs = deque(maxlen=maxlen)
        self.rewards = deque(maxlen=maxlen)
        self.dones = deque(maxlen=maxlen)
        self.values = deque(maxlen=maxlen)
        self.masks = deque(maxlen=maxlen)
    def add(self, state, action, log_prob, reward, done, value, mask):
        """Add a single transition to the buffer."""
        self.states.append(state)
        self.actions.append(action)
        self.log_probs.append(log_prob)
        self.rewards.append(reward)
        self.dones.append(done)
        self.values.append(value)
        self.masks.append(mask)
    def clear(self):
        """Clear all stored transitions."""
        self.states.clear()
        self.actions.clear()
        self.log_probs.clear()
        self.rewards.clear()
        self.dones.clear()
        self.values.clear()
        self.masks.clear()
    def __len__(self):
        return len(self.states)
# PPO Agent
class PPOAgent:
    """
    Proximal Policy Optimization agent for NR MAC scheduling.
    Implements the PPO clip algorithm with Generalised Advantage
    Estimation (GAE) for variance reduction.
    Parameters
    ----------
    num_flows : int
        Maximum number of active flows.
    hidden_dim : int
        Hidden layer size (default 256).
    lr : float
        Learning rate (default 3e-4).
    gamma : float
        Discount factor (default 0.99).
    gae_lambda : float
        GAE lambda parameter (default 0.95).
    eps_clip : float
        PPO clipping epsilon (default 0.2).
    k_epochs : int
        Number of PPO update epochs (default 4).
    entropy_coeff : float
        Entropy bonus coefficient (default 0.01).
    value_coeff : float
        Value loss coefficient (default 0.5).
    """
    def __init__(
        self,
        num_flows,
        hidden_dim=256,
        lr=3e-4,
        gamma=0.99,
        gae_lambda=0.95,
        eps_clip=0.2,
        k_epochs=4,
        entropy_coeff=0.01,
        value_coeff=0.5,
    ):
        self.num_flows = num_flows
        self.gamma = gamma
        self.gae_lambda = gae_lambda
        self.eps_clip = eps_clip
        self.k_epochs = k_epochs
        self.entropy_coeff = entropy_coeff
        self.value_coeff = value_coeff
        self.device = torch.device(
            "cuda" if torch.cuda.is_available() else "cpu"
        )
        print(f"Using device: {self.device}")
        self.policy = ActorCritic(num_flows, NUM_FEATURES, hidden_dim).to(
            self.device
        )
        self.optimizer = optim.Adam(self.policy.parameters(), lr=lr)
        # Old policy for importance sampling ratio
        self.policy_old = ActorCritic(num_flows, NUM_FEATURES, hidden_dim).to(
            self.device
        )
        self.policy_old.load_state_dict(self.policy.state_dict())
        self.mse_loss = nn.MSELoss()
    def select_action(self, state, mask):
        """
        Select scheduling weights for the current TTI.
        Parameters
        ----------
        state : np.ndarray
            Shape (num_flows, NUM_FEATURES).
        mask : list[bool]
            Which flows are active this TTI.
        Returns
        -------
        actions : np.ndarray
            Scheduling weights for each flow.
        log_probs : torch.Tensor
            Log-probabilities (for PPO update).
        value : torch.Tensor
            Value estimate (for GAE).
        """
        with torch.no_grad():
            return self.policy_old.act(state, mask)
    def compute_gae(self, rewards, values, dones):
        """
        Compute Generalised Advantage Estimation.
        Uses the standard GAE formula:
            A_t = sum_{l=0}^{T-t} (gamma * lambda)^l * delta_{t+l}
            delta_t = r_t + gamma * V(s_{t+1}) * (1 - done_{t+1}) - V(s_t)
        Parameters
        ----------
        rewards : list[float]
        values : list[float]
        dones : list[bool]
        Returns
        -------
        advantages : torch.Tensor
        returns : torch.Tensor
        """
        advantages = []
        gae = 0.0
        values_ext = list(values) + [0.0]  # Bootstrapped value at end
        for t in reversed(range(len(rewards))):
            if dones[t]:
                delta = rewards[t] - values_ext[t]
                gae = delta
            else:
                delta = (
                    rewards[t]
                    + self.gamma * values_ext[t + 1]
                    - values_ext[t]
                )
                gae = delta + self.gamma * self.gae_lambda * gae
            advantages.insert(0, gae)
        advantages = torch.tensor(advantages, dtype=torch.float32).to(
            self.device
        )
        returns = advantages + torch.tensor(values, dtype=torch.float32).to(
            self.device
        )
        # Normalise advantages
        if len(advantages) > 1:
            advantages = (advantages - advantages.mean()) / (
                advantages.std() + 1e-8
            )
        return advantages, returns
    def update(self, buffer):
        """
        Perform PPO update using collected experience.
        Parameters
        ----------
        buffer : RolloutBuffer
            Contains the rollout data.
        Returns
        -------
        mean_loss : float
            Average loss over all update epochs.
        """
        # Compute GAE
        values_list = [v.item() for v in buffer.values]
        advantages, returns = self.compute_gae(
            list(buffer.rewards), values_list, list(buffer.dones)
        )
        # Convert buffer to tensors
        old_states = torch.tensor(
            np.array(list(buffer.states)), dtype=torch.float32
        ).to(self.device)
        old_actions = torch.tensor(
            np.array(list(buffer.actions)), dtype=torch.float32
        ).to(self.device)
        old_log_probs_list = []
        for lp in buffer.log_probs:
            if isinstance(lp, torch.Tensor):
                old_log_probs_list.append(lp.detach())
            else:
                old_log_probs_list.append(torch.tensor(lp))
        old_log_probs = torch.stack(old_log_probs_list).to(self.device)
        masks = list(buffer.masks)
        total_loss = 0.0
        for _ in range(self.k_epochs):
            # Evaluate current policy
            log_probs, state_values, dist_entropy = self.policy.evaluate(
                old_states, old_actions, masks
            )
            # Importance sampling ratio
            ratios = torch.exp(
                log_probs.sum(dim=1) - old_log_probs.sum(dim=1)
            )
            # PPO clip surrogate loss
            surr1 = ratios * advantages
            surr2 = (
                torch.clamp(ratios, 1 - self.eps_clip, 1 + self.eps_clip)
                * advantages
            )
            # Total loss: policy + value + entropy bonus
            policy_loss = -torch.min(surr1, surr2).mean()
            value_loss = self.mse_loss(state_values, returns)
            entropy_loss = -dist_entropy
            loss = (
                policy_loss
                + self.value_coeff * value_loss
                + self.entropy_coeff * entropy_loss
            )
            self.optimizer.zero_grad()
            loss.backward()
            # Gradient clipping for stability
            torch.nn.utils.clip_grad_norm_(
                self.policy.parameters(), max_norm=0.5
            )
            self.optimizer.step()
            total_loss += loss.item()
        # Sync old policy
        self.policy_old.load_state_dict(self.policy.state_dict())
        return total_loss / self.k_epochs
    def save(self, path):
        """Save model checkpoint."""
        torch.save(
            {
                "policy_state_dict": self.policy.state_dict(),
                "optimizer_state_dict": self.optimizer.state_dict(),
            },
            path,
        )
        print(f"Model saved to {path}")
    def load(self, path):
        """Load model checkpoint."""
        checkpoint = torch.load(path, map_location=self.device)
        self.policy.load_state_dict(checkpoint["policy_state_dict"])
        self.policy_old.load_state_dict(checkpoint["policy_state_dict"])
        self.optimizer.load_state_dict(checkpoint["optimizer_state_dict"])
        print(f"Model loaded from {path}")
# Shared Memory Helpers
def read_observations(env_msg):
    """
    Read per-flow observations from the shared memory envelope.
    Converts NrSchedEnvMsg fields into a numpy array of shape
    (num_flows, NUM_FEATURES).
    Parameters
    ----------
    env_msg : ns3ai.NrSchedEnvMsg
        The C++ to Python envelope in shared memory.
    Returns
    -------
    obs : np.ndarray
        Shape (MAX_FLOWS, NUM_FEATURES).
    num_flows : int
        Number of active flows this TTI.
    reward : float
        Reward from the previous action.
    is_finished : bool
        Whether the simulation has ended.
    mask : list[bool]
        Which flow slots are active.
    """
    num_flows = env_msg.numFlows
    reward = env_msg.reward
    is_finished = env_msg.isFinished
    obs = np.zeros((MAX_FLOWS, NUM_FEATURES), dtype=np.float32)
    mask = [False] * MAX_FLOWS
    for i in range(min(num_flows, MAX_FLOWS)):
        o = env_msg.get_obs(i)
        obs[i] = [
            float(o.rnti),
            float(o.lcId),
            float(o.fiveQi),
            float(o.priority),
            float(o.holDelay),
            o.cqi,
            o.bsr,
            o.avgTput,
            o.potentialTput,
        ]
        mask[i] = True
    return obs, num_flows, reward, is_finished, mask
def write_actions(act_msg, actions, num_flows, obs):
    """
    Write per-flow scheduling weights to the shared memory envelope.
    Parameters
    ----------
    act_msg : ns3ai.NrSchedActMsg
        The Python -> C++ envelope in shared memory.
    actions : np.ndarray
        Scheduling weights for each flow.
    num_flows : int
        Number of active flows.
    obs : np.ndarray
        Observations (used to copy RNTI and LCID back to actions).
    """
    act_msg.numFlows = num_flows
    for i in range(min(num_flows, MAX_FLOWS)):
        a = act_msg.get_action(i)
        a.rnti = int(obs[i, 0])   # RNTI from observation
        a.lcId = int(obs[i, 1])   # LCID from observation
        a.weight = float(actions[i])
# Main Training Loop
def main(args):
    """
    Main PPO training loop.
    Creates the shared memory interface, attaches to the segment
    created by the C++ simulation, and runs the PPO agent loop:
    1. Wait for C++ to send observations
    2. Read observations + reward from shared memory
    3. Select actions via the actor network
    4. Write weights to shared memory
    5. Signal C++ to continue
    6. Periodically update the policy
    Parameters
    ----------
    args : argparse.Namespace
        CLI arguments.
    """
    print("=" * 60)
    print("NR AI Scheduler — PPO Agent (msg-interface)")
    print("=" * 60)
    print(f"  num_flows:   {args.num_flows}")
    print(f"  hidden_dim:  {args.hidden_dim}")
    print(f"  lr:          {args.lr}")
    print(f"  gamma:       {args.gamma}")
    print(f"  gae_lambda:  {args.gae_lambda}")
    print(f"  eps_clip:    {args.eps_clip}")
    print(f"  k_epochs:    {args.k_epochs}")
    print(f"  update_interval: {args.update_interval}")
    print("=" * 60)
    # Create shared memory interface (Python side = NOT memory creator)
    mem_size = 4096  # Must be >= sizeof(NrSchedEnvMsg) + sizeof(NrSchedActMsg)
    msg_interface = ns3ai.Ns3AiMsgInterfaceImpl(
        False,          # is_memory_creator = False (C++ creates)
        False,          # use_vector = False (struct mode)
        True,           # handle_finish = True
        mem_size,
        SHM_SEGMENT,
        SHM_CPP2PY,
        SHM_PY2CPP,
        SHM_LOCK,
    )
    print("Attached to shared memory segment:", SHM_SEGMENT)
    # Create PPO agent
    agent = PPOAgent(
        num_flows=args.num_flows,
        hidden_dim=args.hidden_dim,
        lr=args.lr,
        gamma=args.gamma,
        gae_lambda=args.gae_lambda,
        eps_clip=args.eps_clip,
        k_epochs=args.k_epochs,
    )
    if args.load_model and os.path.exists(args.load_model):
        agent.load(args.load_model)
    buffer = RolloutBuffer(maxlen=args.update_interval)
    # Reward logging
    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    log_file = log_dir / "rewards.csv"
    csv_file = open(log_file, "w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["step", "reward", "num_flows", "loss"])
    # Training loop
    step = 0
    total_reward = 0.0
    update_count = 0
    print("\nWaiting for C++ simulation to start...")
    try:
        while True:
            # STEP 1: Wait for C++ to send observations
            msg_interface.PyRecvBegin()
            # Check if simulation is done
            if msg_interface.PyGetFinished():
                msg_interface.PyRecvEnd()
                print("\nSimulation finished signal received.")
                break
            # STEP 2: Read observations from shared memory
            env_msg = msg_interface.GetCpp2PyStruct()
            obs, num_flows, reward, is_finished, mask = read_observations(
                env_msg
            )
            msg_interface.PyRecvEnd()
            if is_finished:
                print("\nSimulation episode ended.")
                break
            total_reward += reward
            # STEP 3: Select actions via policy
            actions, log_probs, value = agent.select_action(obs, mask)
            # STEP 4: Write actions to shared memory
            msg_interface.PySendBegin()
            act_msg = msg_interface.GetPy2CppStruct()
            write_actions(act_msg, actions, num_flows, obs)
            msg_interface.PySendEnd()
            # STEP 5: Store transition in buffer
            # (reward is for the PREVIOUS action, so store it with
            #  the previous state — here we store current for simplicity
            #  and the reward arrives next step)
            buffer.add(
                state=obs.copy(),
                action=actions.copy(),
                log_prob=log_probs,
                reward=reward,
                done=is_finished,
                value=value,
                mask=mask.copy(),
            )
            # STEP 6: PPO update when buffer is full
            if len(buffer) >= args.update_interval:
                loss = agent.update(buffer)
                update_count += 1
                avg_reward = total_reward / args.update_interval
                print(
                    f"  Update {update_count}: "
                    f"avg_reward={avg_reward:.4f}, "
                    f"loss={loss:.4f}, "
                    f"flows={num_flows}"
                )
                csv_writer.writerow([step, avg_reward, num_flows, loss])
                csv_file.flush()
                buffer.clear()
                total_reward = 0.0
                # Checkpoint every N updates
                if (
                    args.save_interval > 0
                    and update_count % args.save_interval == 0
                ):
                    ckpt_path = log_dir / f"ppo_ckpt_{update_count}.pt"
                    agent.save(str(ckpt_path))
            step += 1
            if step % 500 == 0:
                print(
                    f"  Step {step}: "
                    f"reward={reward:.4f}, "
                    f"flows={num_flows}"
                )
    except KeyboardInterrupt:
        print("\n\nTraining interrupted by user (Ctrl+C).")
    finally:
        # Save final model
        final_path = log_dir / "ppo_final.pt"
        agent.save(str(final_path))
        csv_file.close()
        print(f"\nReward log saved to: {log_file}")
        print(f"Final model saved to: {final_path}")
        print(f"Total steps: {step}, Updates: {update_count}")
        print("Done.")
# CLI
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="PPO agent for NR AI scheduler (msg-interface)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    # Agent hyperparameters
    parser.add_argument(
        "--num_flows",
        type=int,
        default=MAX_FLOWS,
        help="Maximum number of flows (must match C++ MAX_FLOWS)",
    )
    parser.add_argument(
        "--hidden_dim",
        type=int,
        default=256,
        help="Hidden layer dimension for Actor-Critic",
    )
    parser.add_argument(
        "--lr",
        type=float,
        default=3e-4,
        help="Learning rate for Adam optimiser",
    )
    parser.add_argument(
        "--gamma",
        type=float,
        default=0.99,
        help="Discount factor",
    )
    parser.add_argument(
        "--gae_lambda",
        type=float,
        default=0.95,
        help="GAE lambda parameter",
    )
    parser.add_argument(
        "--eps_clip",
        type=float,
        default=0.2,
        help="PPO clipping epsilon",
    )
    parser.add_argument(
        "--k_epochs",
        type=int,
        default=4,
        help="Number of PPO update epochs per batch",
    )
    parser.add_argument(
        "--update_interval",
        type=int,
        default=1000,
        help="Number of TTI steps between PPO updates",
    )
    # Logging and checkpointing
    parser.add_argument(
        "--log_dir",
        type=str,
        default="./nr_ai_sched_logs",
        help="Directory for reward logs and model checkpoints",
    )
    parser.add_argument(
        "--save_interval",
        type=int,
        default=10,
        help="Save checkpoint every N updates (0 = never)",
    )
    parser.add_argument(
        "--load_model",
        type=str,
        default="",
        help="Path to a saved model checkpoint to resume training",
    )
    args = parser.parse_args()
    main(args)