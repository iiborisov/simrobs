# simrobs
# Simulation of Robotic Systems 
Students: Robotics and artificial intelligence

Instructor: Ivan Borisov, PhD, borisovii@itmo.ru 

Assistant: Egor Rakshin, MSc, earakshin@itmo.ru

## Description
Modern robotic and mechatronic systems are complex in terms of all domains: mechanics, sensors, actuation, control, etc. To study the behavior and performance of an existing robotic system or a proposed one, we use models to focus on the essential features while keeping a reasonable tradeoff between realism and simplicity.  The act of building a model is called modeling, while the process of using a model to study the behavior and performance of an actual or theoretical system is called simulation.

This module discusses modern techniques for creating and using models to study the behavior and performance of robotic systems. The module consists of lecture and practice parts. The lectures mostly give theoretical inputs on modeling such as screw theory & Lie groups to describe motion, bond-graphs to describe the interconnection of different physical systems, control strategies to steer the systems, and optimization procedures for mechanics and control. The practice part focuses on simulation skills using MuJoCo and Pinocchio.

## **Learning objectives**

After the module the student:

1. Understands trade-offs when modeling dynamical systems
2. Understands advanced Screw Theory & Lie Groups and Bond graph modeling concepts 
3. Understands how to use simulation to gain insight into, analyze, and optimize models
4. Is able to use MuJoCo for multibody simulation

## **Materials**

- Lecture presentations
    - [Video lectures in English](https://youtube.com/playlist?list=PLERoBxyD-nQgp4CSGcG_UCeCYBanRjSKw)
    - [Video lectures in Russian](https://youtube.com/playlist?list=PLERoBxyD-nQgGCDoGzwNDXNfGAjK8wKV8)
- "Modern Robotics: Mechanics, Planning, and Control," Kevin M. Lynch and Frank C. Park, Cambridge University Press, 2017
- R. M. Murray, Z. Li, S. S. Sastry, and S. S. Sastry, A mathematical introduction to robotic manipulation. CRC press, 1994.
- Featherstone, Roy. Rigid body dynamics algorithms. Springer, 2014.

# Environment installation 
First, you need to install `conda` package manager. Use one of the suggested [instructions](https://docs.anaconda.com/anaconda/install/) depending on your OS.

After installing conda, set up the virtual environment by running the following command in the terminal
```bash
conda env create -f environment.yml
```
Next, activate the environment:  
```bash
conda activate mds
```

More information about virtual environments you can find [here](https://docs.anaconda.com/working-with-conda/environments/).
