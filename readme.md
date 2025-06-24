**Blending Task and Motion Planning using Learning from Demonstration and Reinforcement Learning**


Task and motion planning (TAMP) deals with complex tasks that require the execution of multiple actions in a chronological order and the ability to generalize to variable object configurations. Symbolic planning efficiently generates task plans of multiple symbolic actions. This thesis focuses on grounding these symbolic actions such that feasible motion is executed in varying scenarios. Therefore, an initial trajectory is learned from one demonstration and subsequentlly diversified with RL. A neural network is trained to represent the action policy and to generate collision-free trajectories in varying scenarios. The framework is applied to a sequential task of rearranging cubes from a random initial configuration into a random goal configuration. The image shows a screenshot of one step in the task plan where the policy generates a trajectory that avoids a collision with cube4 and cube6.

![TAMP_simluation](https://github.com/domi20u/Projects/blob/master/TAMP%20using%20LfD%20%26%20RL/TAMP_sorting_cubes.jpg)
