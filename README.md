<p align="center">
  <img src="https://www.svgrepo.com/show/397430/mechanical-arm.svg" width="100" alt="project-logo">
</p>
<p align="center">
    <h1 align="center">EVA Armbot Control Class</h1>
</p>
<p align="center">
    <em><code>EVA (Automata Robot Arms) – Simplified user control class. "Armbot" control class developed in Python for two Automata "Eva" robot arms, using Automata EVA API calls, solving forward and inverse kinematics for the end efector position by applying lagrangian-jacobian matrix computation. Using Python.</code></em>
</p>

<br><!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary><br>

- [ Overview](#overview)
- [ Project Files](#project-files)
- [ Repository Structure](#repository-structure)
- [ Modules](#modules)
- [ Getting Started](#getting-started)
  - [ Installation](#installation)
  - [ Usage](#usage)
- [ Key Components](#key-components)
- [ Contributing](#contributing)
</details>
<hr>

##  Overview

<code>This project provides a Python-based control class for managing two Automata "Eva" robot arms. The control class leverages Automata's EVA API to interact with the robots, solving forward and inverse kinematics problems. The solutions are based on the Lagrangian-Jacobian matrix computation method. This method is essential for calculating the positions of the robot arms' end effectors—i.e., the parts of the arms that interact with the environment. By using the provided scripts, users can effectively control the robot arms for various applications.</code>

---

##  Project Files

Here is a brief overview of the critical files and their purposes:

* `armbot.py`: Contains the main class for robot arm control.
* `kinematics.py`: Implements the kinematics algorithms using Lagrangian-Jacobian matrix computations.
* Several test files (`armbot class test 1.py` to `armbot class test 5.py`): These scripts demonstrate various use cases of the `Armbot` class.
* `reset_position.py`: A utility script to reset the robot arms to their initial positions.
* `sample code 1.py` to `sample code 4.py`: Additional examples showing different functionalities of the robot arms.

---

##  Repository Structure

```sh
└── eva_armbot_control_class/
    ├── README.md
    ├── aravis.py
    ├── armbot class test 1.py
    ├── armbot class test 2.py
    ├── armbot class test 3.py
    ├── armbot class test 4.py
    ├── armbot class test 5.py
    ├── armbot.py
    ├── evasdk
    │   ├── Eva.py
    │   ├── EvaDiscoverer.py
    │   ├── __init__.py
    │   ├── __pycache__
    │   ├── eva_errors.py
    │   ├── eva_http_client.py
    │   ├── eva_locker.py
    │   ├── eva_ws.py
    │   ├── helpers.py
    │   ├── robot_state.py
    │   └── version.py
    ├── hello arm 2.py
    ├── hello arm.py
    ├── image.png
    ├── kinematics.py
    ├── reset_position.py
    ├── sample code 1.py
    ├── sample code 2.py
    ├── sample code 3.py
    └── sample code 4.py
```

---

##  Modules

<details closed><summary>.</summary>

| File                                                                                                                       | Summary                         |
| ---                                                                                                                        | ---                             |
| [reset_position.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/reset_position.py)           | <code>► INSERT-TEXT-HERE</code> |
| [armbot class test 4.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/armbot class test 4.py) | <code>► INSERT-TEXT-HERE</code> |
| [armbot class test 1.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/armbot class test 1.py) | <code>► INSERT-TEXT-HERE</code> |
| [hello arm 2.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/hello arm 2.py)                 | <code>► INSERT-TEXT-HERE</code> |
| [sample code 3.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/sample code 3.py)             | <code>► INSERT-TEXT-HERE</code> |
| [armbot.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/armbot.py)                           | <code>► INSERT-TEXT-HERE</code> |
| [armbot class test 3.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/armbot class test 3.py) | <code>► INSERT-TEXT-HERE</code> |
| [kinematics.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/kinematics.py)                   | <code>► INSERT-TEXT-HERE</code> |
| [sample code 4.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/sample code 4.py)             | <code>► INSERT-TEXT-HERE</code> |
| [aravis.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/aravis.py)                           | <code>► INSERT-TEXT-HERE</code> |
| [sample code 2.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/sample code 2.py)             | <code>► INSERT-TEXT-HERE</code> |
| [armbot class test 2.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/armbot class test 2.py) | <code>► INSERT-TEXT-HERE</code> |
| [hello arm.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/hello arm.py)                     | <code>► INSERT-TEXT-HERE</code> |
| [armbot class test 5.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/armbot class test 5.py) | <code>► INSERT-TEXT-HERE</code> |
| [sample code 1.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/sample code 1.py)             | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>evasdk</summary>

| File                                                                                                                      | Summary                         |
| ---                                                                                                                       | ---                             |
| [eva_http_client.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/evasdk/eva_http_client.py) | <code>► INSERT-TEXT-HERE</code> |
| [eva_ws.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/evasdk/eva_ws.py)                   | <code>► INSERT-TEXT-HERE</code> |
| [EvaDiscoverer.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/evasdk/EvaDiscoverer.py)     | <code>► INSERT-TEXT-HERE</code> |
| [eva_locker.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/evasdk/eva_locker.py)           | <code>► INSERT-TEXT-HERE</code> |
| [eva_errors.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/evasdk/eva_errors.py)           | <code>► INSERT-TEXT-HERE</code> |
| [version.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/evasdk/version.py)                 | <code>► INSERT-TEXT-HERE</code> |
| [Eva.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/evasdk/Eva.py)                         | <code>► INSERT-TEXT-HERE</code> |
| [helpers.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/evasdk/helpers.py)                 | <code>► INSERT-TEXT-HERE</code> |
| [robot_state.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/evasdk/robot_state.py)         | <code>► INSERT-TEXT-HERE</code> |

</details>

---

##  Getting Started

**Prerequisites:**

To run this project, you need Python installed on your system (Python 3.6 or newer is recommended). Additionally, you must install the following libraries:

* **Python**: `version 3.6.0++`
* **NumPy**: `For numerical calculations`
* **Matplotlib**: `For plotting and visualization of the robot's movements`
* **SymPy**: `For symbolic mathematics and kinematics calculations`
* **OpenCV (cv2)**: `For image processing and computer vision tasks`
* **Threading**: `For parallel execution of image acquisition and processing`
* **evasdk (specific to the EVA robotic arm, included in evasdk folder)**: `For eva robot arm control`
* **aravis**: `For camera control`

You can install these packages using pip:

> ```console
> $ pip install numpy matplotlib sympy opencv-python aravis
> ```

###  Installation

<h4>From <code>source</code></h4>

> 1. Clone the eva_armbot_control_class repository:
>
> ```console
> $ git clone https://github.com/Alexpascual28/eva_armbot_control_class.git
> ```
>
> 2. Change to the project directory:
> ```console
> $ cd eva_armbot_control_class
> ```
>
> 3. Install the dependencies:
> ```console
> $ pip install -r requirements.txt
> ```

###  Usage

To test the Armbot class with one of the test scripts, follow these steps:

Ensure your environment is set up with all necessary dependencies.
Navigate to the project directory.
Run a test script:

> ```console
> $ python armbot_class_test_1.py
> ```

## Key Components

### Kinematics: Mathematical Foundations

`kinematics.py` defines the **Kinematics** class responsible for calculating the robot arm's movements. It uses mathematical models to determine how the arm should move its joints to reach a specific position.

**Inverse Kinematics and the Jacobian Matrix**

Inverse kinematics is the process of determining the joint parameters that provide a desired position of the robot's end effector. The Jacobian matrix is a critical tool in solving inverse kinematics problems because it relates the change in joint angles to the resulting movement of the end effector in space.

In the Kinematics class, we calculate the Jacobian matrix as follows:

> ```python
> # Calculate the numerical value of J (jacobian) at each point,
> # you can produce the Jacobian as follows
> J = p.jacobian(theta)
> J_i = J.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2],
>             theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
> ```

Where `p` is the position vector of the end effector, and `theta` represents the joint angles.

To move the end effector towards a target, we adjust the joint angles based on the calculated differential movement `dp`. The Jacobian inverse is used to find the necessary change in joint angles `dtheta` to achieve `dp`:

> ```python
> J_inv = J_i.pinv()
> dtheta = J_inv * dp_step
> ```

`dp_step` is the desired small step towards the target position. The pseudoinverse (`pinv()`) of the Jacobian is used when the matrix is not square, ensuring a solution exists.

### ArmBot

The **ArmBot** class, defined in `armbot.py`, using the kinematics class to control the robotic arm and integrates computer vision for object detection. To start the class you must first create an instance of it by providing the arm name as follows:

> ```python
> arm = Arm("evatrendylimashifterpt410")
> ```

The arm and camera names and ips can be changed in the "initialize_arm()" and the "initialize_camera()" functions within the armbot class.

After an arm object has been instatiated, functions can be run using dot notation as usual. Key functions include:

**Main Functions of ArmBot Class**

* **move_end_efector(absolute_position)**: Moves the end effector to an absolute position.
* **shift_end_efector(relative_position)**: Adjusts the end effector's position relative to its current location.
* **set_joint_angles(joint_angles)**: Moves arm by directly commanding each joint to move to the specified angle (6DOF).
* **open_gripper()** and **close_gripper()**: Controls the gripper to pick up or release objects.
* **start_image_acquisition()**: Starts continuous image acquisition and processing, in parallel thread.
* **get_current_image()**: Returns current image in continuous feed thread.
* **take_picture()**: Takes and returns single picture without having to start a parallel continuous feed.
* **detect_colour(image, colour_name)**: Identifies objects in the image based on their color.
* **detect_shapes(mask, shape_name)**: Detects geometric shapes within a masked image.

---

##  Contributing

Contributions are welcome! Here are several ways you can contribute:

- **[Report Issues](https://github.com/Alexpascual28/eva_armbot_control_class.git/issues)**: Submit bugs found or log feature requests for the `eva_armbot_control_class` project.
- **[Submit Pull Requests](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/main/CONTRIBUTING.md)**: Review open PRs, and submit your own PRs.
- **[Join the Discussions](https://github.com/Alexpascual28/eva_armbot_control_class.git/discussions)**: Share your insights, provide feedback, or ask questions.

<details closed>
<summary>Contributing Guidelines</summary>

1. **Fork the Repository**: Start by forking the project repository to your github account.
2. **Clone Locally**: Clone the forked repository to your local machine using a git client.
   ```sh
   git clone https://github.com/Alexpascual28/eva_armbot_control_class.git
   ```
3. **Create a New Branch**: Always work on a new branch, giving it a descriptive name.
   ```sh
   git checkout -b new-feature-x
   ```
4. **Make Your Changes**: Develop and test your changes locally.
5. **Commit Your Changes**: Commit with a clear message describing your updates.
   ```sh
   git commit -m 'Implemented new feature x.'
   ```
6. **Push to github**: Push the changes to your forked repository.
   ```sh
   git push origin new-feature-x
   ```
7. **Submit a Pull Request**: Create a PR against the original project repository. Clearly describe the changes and their motivations.
8. **Review**: Once your PR is reviewed and approved, it will be merged into the main branch. Congratulations on your contribution!
</details>

<details closed>
<summary>Contributor Graph</summary>
<br>
<p align="center">
   <a href="https://github.com{/Alexpascual28/eva_armbot_control_class.git/}graphs/contributors">
      <img src="https://contrib.rocks/image?repo=Alexpascual28/eva_armbot_control_class.git">
   </a>
</p>
</details>

---
