# simrobs
Repository for the course "Simulation of Robotic Systems"

Use the following command to install enviroment
```bash
   conda env create -f environment.yml
```

Make sure that conda-forge channel is accessible. Use `conda config --show channels` to view your configuration's current state,
and use `conda config --show-sources` to view config file locations. Use  `conda config --add channels conda-forge` To enable conda-forge channel.

Robots are programmable machines to implement the desired action. Like any machine, a robot is needed to perform an automatic mechanical motion instead of a human, in other words, to make humans' lives easier. A robot uses external energy to apply forces and torques to transform the motion of actuated input links to get the desired output links' motion. There is a vast range of robotics systems for different applications. The hardware and software of the robots' significantly differ depending on their purposes.