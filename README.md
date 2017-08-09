# Intrinsically Motivated Goal Exploration Processes with Automatic Curriculum Learning
by Sébastien Forestier, Yoan Mollard and Pierre-Yves Oudeyer

The paper is available on arXiv: https://arxiv.org/abs/1708.02190

A video explains the algorithms and shows the exploration and learning on the robotic setup: https://youtu.be/NOLAwD4ZTW0

Abstract:

Intrinsically motivated spontaneous exploration is a key enabler of autonomous lifelong learning in human children. It allows them to discover and acquire large repertoires of skills through self-generation, self-selection, self-ordering and self-experimentation of learning goals. We present the unsupervised multi-goal reinforcement learning formal framework as well as an algorithmic approach called intrinsically motivated goal exploration processes (IMGEP) to enable similar properties of autonomous learning in machines. The IMGEP algorithmic architecture relies on several principles: 1) self-generation of goals as parameterized reinforcement learning problems; 2) selection of goals based on intrinsic rewards; 3) exploration with parameterized time-bounded policies and fast incremental goal-parameterized policy search; 4) systematic reuse of information acquired when targeting a goal for improving other goals. We present a particularly efficient form of IMGEP that uses a modular representation of goal spaces as well as intrinsic rewards based on learning progress. We show how IMGEPs automatically generate a learning curriculum within an experimental setup where a real humanoid robot can explore multiple spaces of goals with several hundred continuous dimensions. While no particular target goal is provided to the system beforehand, this curriculum allows the discovery of skills of increasing complexity, that act as stepping stone for learning more complex skills (like nested tool use). We show that learning several spaces of diverse problems can be more efficient for learning complex skills than only trying to directly learn these complex skills. We illustrate the computational efficiency of IMGEPs as these robotic experiments use a simple memory-based low-level policy representations and search algorithm, enabling the whole system to learn online and incrementally on a Raspberry Pi 3. 

## Open-source and open-hardware
This repository gives the open-source code of the experiments described in the paper, together with 3D shapes of the parts that need to be 3D-printed. The code is based on [ROS](http://www.ros.org/) for the communication between robots / webcam etc.

## References
This research project is developed by the Flowers team at Inria and Ensta ParisTech: https://flowers.inria.fr

Here are some links:

A Python/Explauto Tutorial: http://nbviewer.jupyter.org/github/sebastien-forestier/ExplorationAlgorithms/blob/master/main.ipynb

A Paper on the Active Model Babbling architecture: Forestier S, Oudeyer P-Y. 2016. Modular Active Curiosity-Driven Discovery of Tool Use. 2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). http://sforestier.com/sites/default/files/Forestier2016Modular.pdf

Poppy Project: an open-source 3D printed low-cost humanoid robotic platform that allows non-roboticists to quickly set up and program robotic experiments. https://www.poppy-project.org

Explauto: an open-source Python library to benchmark active learning and exploration algorithms that includes already implemented real and simulated robotics setups and exploration algorithms. https://github.com/flowersteam/explauto

The poster of the demo: https://hal.inria.fr/hal-01404399/

This project was conducted within a larger long-term research program at the Flowers lab on mechanisms of lifelong learning and development in machines and humans. This research program has in particular lead to a series of novel intrinsically motivated learning algorithms working on high-dimensional real robots and opening new perspectives in cognitive sciences. Papers providing this broader context are:

Oudeyer, P-Y. and Smith. L. (2016) How Evolution may work through Curiosity-driven Developmental Process, Topics in Cognitive Science, 1-11. https://hal.inria.fr/hal-01404334

P-Y. Oudeyer, J. Gottlieb and M. Lopes (2016) Intrinsic motivation, curiosity and learning: theory and applications in educational technologies, Progress in Brain Research, 229, pp. 257-284. https://hal.inria.fr/hal-01404278

Baranes, A., Oudeyer, P-Y. (2013) Active Learning of Inverse Models with Intrinsically Motivated Goal Exploration in Robots, Robotics and Autonomous Systems, 61(1), pp. 49-73. https://hal.inria.fr/hal-00788440

Oudeyer P-Y. and Kaplan F. (2007) What is intrinsic motivation? A typology of computational approaches. Frontiers in Neurorobotics, 1:6. http://journal.frontiersin.org/article/10.3389/neuro.12.006.2007/full
