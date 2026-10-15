# Example of some ppo and how to properly wire a PyBullet simulation into a Gymnasium environment for RL training


Custom Gym Environments
Some custom Gym environments for reinforcement learning.
https://github.com/PierreExeter/custom_gym_envs


Reinforcement Learning with the Franka Panda robot pybullet simulation
This is an example showing how to train a Reinforcement Learning agent for the Franka Panda robot arm using:
https://github.com/simonbogh/rl_panda_gym_pybullet_example
- PyBullet physics simulation
- Gymnasium (formerly OpenAI gym) RL API
- panda-gym robot environments in PyBullet
- Stable-Baselines3 RL algorithms library


he goal is to coordinate the four legs to move in the forward (right) 
direction by applying torque to the eight hinges connecting the two body parts of 
each leg and the torso (nine body parts and eight hinges).
https://gymnasium.farama.org/environments/mujoco/ant/


Basic Usage: Training, Saving, Loading
In the following example, we will train, save and load a DQN model on the Lunar Lander environment, really usefull 
https://stable-baselines3.readthedocs.io/en/master/guide/examples.html#basic-usage-training-saving-loading
https://stable-baselines3.readthedocs.io/en/master/guide/examples.html#multiprocessing-unleashing-the-power-of-vectorized-environments
