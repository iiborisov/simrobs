# simrobs

Lecture notes, interactive presentations, and Python examples for two subjects taught at ITMO University for Robotics and Artificial Intelligence students.

**SRS 2026 / Моделирование робототехнических систем:** [course catalog](SRS_2026/index.html) · [materials and publication instructions](SRS_2026/README.md). All six lectures have Russian and English notes and slides.

Instructor: Ivan Borisov, PhD, borisovii@itmo.ru

Assistant: Egor Rakshin, PhD, earakshin@itmo.ru

# DOMS: Design and Optimization of Mechatronic Systems

## Description

The design process is often not formalized and mostly relies on the designer’s imagination, engineering intuition,
and experience. Within this subject we utilize computational design paradigm, that treats the design process as
an optimization task, to optimize mechanics and control for robotic and mechatronic systems.
Computational Design consists of two tasks: the user-driven the forward exploration to compute performance and behavior of
a system with a given set of parameters and the optimization-driven inverse task to determine values that lead to a desired
motion or behavior. The module discuss how computational design approach can be used for engineering purposes.

## **Materials**

- Lecture notes
- Book: Hurbans, Rishal. "Artificial Intelligence Algorithms." Hurbans Rishal–Shelter Island, NY 11964 (2021) [Link](https://livebook.manning.com/book/grokking-artificial-intelligence-algorithms/chapter-1/).
- Video course: [Introduction to Computational Thinking](https://ocw.mit.edu/courses/18-s191-introduction-to-computational-thinking-fall-2020/)
- Video course: [Introduction to Computational Thinking and Data Science](https://ocw.mit.edu/courses/6-0002-introduction-to-computational-thinking-and-data-science-fall-2016/)
- Papers: [Computational Robotics Lab](https://crl.ethz.ch/publications/index.html)
- Papers: [Disney Research](https://la.disneyresearch.com/publication/)

# SRS: Simulation of Robotic Systems

## Description
Modern robotic and mechatronic systems are complex in terms of all domains: mechanics, sensors, actuation, control, etc. To study the behavior and performance of an existing robotic system or a proposed one, we use models to focus on the essential features while keeping a reasonable tradeoff between realism and simplicity.  The act of building a model is called modeling, while the process of using a model to study the behavior and performance of an actual or theoretical system is called simulation.

The 2026 course develops models from a vertical mass–spring–damper system to planar 2R and spatial 3R robots. Topics include analytical and numerical dynamics, bond graphs, feedback linearization, impedance/admittance control, SO(3)/SE(3), twists, wrenches, and Newton–Euler dynamics. Python examples use NumPy, Matplotlib, and the official MuJoCo bindings.

## Learning objectives

After completing the six lectures, students can:

1. Choose modelling assumptions and distinguish model creation, simulation, verification, and validation.
2. Derive equations of motion and compare forward Euler, backward Euler, and RK4 with an exact solution.
3. Formulate forward/inverse kinematics and dynamics, and model energy exchange using bond graphs.
4. Implement and compare computed-torque, impedance, and admittance controllers.
5. Work with coordinate transformations, twists, wrenches, Jacobians, and spatial inertia.
6. Check models and controllers using independent analytical calculations and MuJoCo experiments.

## SRS 2026 lectures — Русский / English

[Open the HTML course catalog](SRS_2026/index.html). Each lecture includes notes, interactive slides, and examples for a 90-minute class. HTML notes include an outline and Print/PDF controls; the slides include interactive plots and animations. A browser is sufficient for reading them; Python is needed only to run the examples.

| Lecture | Notes RU / EN | Slides RU / EN | Python examples |
|---|---|---|---|
| 1. Modelling and simulation | [RU](SRS_2026/lecture_1_modelling_n_simulation/lecture_notes.md) / [EN](SRS_2026/lecture_1_modelling_n_simulation/lecture_notes.en.md) | [RU](SRS_2026/lecture_1_modelling_n_simulation/presentation/index.html) / [EN](SRS_2026/lecture_1_modelling_n_simulation/presentation/index.en.html) | [Mass–spring–damper](SRS_2026/lecture_1_modelling_n_simulation/mass_spring_damper/README.md) |
| 2. Dynamics of a planar 2R robot | [RU](SRS_2026/lecture_2_2R_robot/lecture_notes.md) / [EN](SRS_2026/lecture_2_2R_robot/lecture_notes.en.md) | [RU](SRS_2026/lecture_2_2R_robot/presentation/index.html) / [EN](SRS_2026/lecture_2_2R_robot/presentation/index.en.html) | [2R tracking](SRS_2026/lecture_2_2R_robot/two_link_robot/README.md) |
| 3. Bond graphs | [RU](SRS_2026/lecture_3_bond_graphs/lecture_notes.md) / [EN](SRS_2026/lecture_3_bond_graphs/lecture_notes.en.md) | [RU](SRS_2026/lecture_3_bond_graphs/presentation/index.html) / [EN](SRS_2026/lecture_3_bond_graphs/presentation/index.en.html) | [Energy-based models](SRS_2026/lecture_3_bond_graphs/examples/README.md) |
| 4. Control of a 2R robot | [RU](SRS_2026/lecture_4_control/lecture_notes.md) / [EN](SRS_2026/lecture_4_control/lecture_notes.en.md) | [RU](SRS_2026/lecture_4_control/presentation/index.html) / [EN](SRS_2026/lecture_4_control/presentation/index.en.html) | [Control experiments](SRS_2026/lecture_4_control/control_2r/README.md) |
| 5. Spatial kinematics | [RU](SRS_2026/lecture_5_spatial_kinematics/lecture_notes.md) / [EN](SRS_2026/lecture_5_spatial_kinematics/lecture_notes.en.md) | [RU](SRS_2026/lecture_5_spatial_kinematics/presentation/index.html) / [EN](SRS_2026/lecture_5_spatial_kinematics/presentation/index.en.html) | [Geometry and 3R](SRS_2026/lecture_5_spatial_kinematics/spatial_kinematics/README.md) |
| 6. Spatial dynamics | [RU](SRS_2026/lecture_6_spatial_dynamics/lecture_notes.md) / [EN](SRS_2026/lecture_6_spatial_dynamics/lecture_notes.en.md) | [RU](SRS_2026/lecture_6_spatial_dynamics/presentation/index.html) / [EN](SRS_2026/lecture_6_spatial_dynamics/presentation/index.en.html) | [Rigid-body and 3R dynamics](SRS_2026/lecture_6_spatial_dynamics/spatial_dynamics/README.md) |

Original PDF lectures and earlier practical materials remain in [SRS](SRS/). Some older examples use Pinocchio, Meshcat, or URDF tools; these are not dependencies of the six `SRS_2026` lectures and are not included in the base environment.

For online access, [GitHub Pages publication instructions](SRS_2026/README.md#публикация-через-github-pages) describe the supplied workflow. The expected address after a successful deployment is `https://iiborisov.github.io/simrobs/SRS_2026/`.

## **Materials**

- Lecture presentations
    - [Video lectures in English](https://youtube.com/playlist?list=PLERoBxyD-nQgp4CSGcG_UCeCYBanRjSKw)
    - [Video lectures in Russian](https://youtube.com/playlist?list=PLERoBxyD-nQgGCDoGzwNDXNfGAjK8wKV8)
- "Modern Robotics: Mechanics, Planning, and Control," Kevin M. Lynch and Frank C. Park, Cambridge University Press, 2017
- R. M. Murray, Z. Li, S. S. Sastry, and S. S. Sastry, A mathematical introduction to robotic manipulation. CRC press, 1994.
- Featherstone, Roy. Rigid body dynamics algorithms. Springer, 2014.

# Environment installation

Install Conda or Miniforge, then run these commands from the repository root. The environment filename is **`enviroment.yml`**.

```bash
conda env create -f enviroment.yml
conda activate simrobs
```

To update an existing environment:

```bash
conda env update -n simrobs -f enviroment.yml
conda activate simrobs
```

The environment contains Python 3.13, NumPy 2.x, Matplotlib, JupyterLab, ipykernel, and MuJoCo. It covers all six `SRS_2026` lectures. MuJoCo's built-in `mujoco.viewer` is used for interactive windows; the separate `mujoco-python-viewer` package is not needed. Per-example `requirements.txt` files describe smaller installations if Conda is not used.

Check the main imports:

```bash
python -c "import numpy, matplotlib, mujoco; print('NumPy:', numpy.__version__, 'Matplotlib:', matplotlib.__version__, 'MuJoCo:', mujoco.__version__)"
```

## Run the examples

Run the following from the repository root with `simrobs` active. Full options, exercises, units, and expected results are in each example's README linked above.

```bash
# Lecture 1: compare three ODE integrators and save plots.
python SRS_2026/lecture_1_modelling_n_simulation/mass_spring_damper/experiments.py --scenario all --plot

# Lecture 2: compare PD and gravity-compensated PD in MuJoCo.
python SRS_2026/lecture_2_2R_robot/two_link_robot/simulate.py --controller compare

# Lecture 3: mechanical, electrical, and electromechanical bond-graph models.
python SRS_2026/lecture_3_bond_graphs/examples/simulate.py --model all --plot

# Lecture 4: computed torque, impedance, and admittance control.
python SRS_2026/lecture_4_control/control_2r/simulate.py

# Lecture 5: spatial geometry and MuJoCo kinematics checks.
python SRS_2026/lecture_5_spatial_kinematics/spatial_kinematics/examples.py
python SRS_2026/lecture_5_spatial_kinematics/spatial_kinematics/mujoco_kinematics.py

# Lecture 6: spatial inertia, rigid-body dynamics, and 3R experiments.
python SRS_2026/lecture_6_spatial_dynamics/spatial_dynamics/inspect_model.py
python SRS_2026/lecture_6_spatial_dynamics/spatial_dynamics/simulate.py
```

Simulation commands above run without opening viewer windows. Add the viewer options documented in the example READMEs to inspect the robots interactively; a graphical desktop is required. On macOS, launch scripts using `mujoco.viewer` with `mjpython` as described in the MuJoCo examples. Outputs are written to each example's `results/` directory unless an output path is supplied.

For notebooks, run `jupyter lab` and select the `simrobs` kernel. If it is not listed, register it:

```bash
python -m ipykernel install --user --name simrobs --display-name "Python (simrobs)"
```
