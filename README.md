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

**SDKs:**

* `evasdk/`: The SDK to use the EVA control class. Uses API calls to interface with the robot arms via ethernet.
* `aravis.py`: The SDK to use the network cameras. This must be placed in the same directory as the `armbot` class being utilized.

**Control Classes**

* `armbot.py`: Contains the main class for robot arm control.
* `kinematics.py`: Implements the kinematics algorithms using Lagrangian-Jacobian matrix computations.

**Tests and Samples**

* `samples/`: Directory containing examples and tests for both the kinematics and the `armbot` control class.
* `armbot/`: Several test files (`armbot class test 1.py` to `armbot class test 5.py`). These scripts demonstrate various use cases of the `Armbot` class.
* `kinematics/`: * `sample code 1.py` to `sample code 4.py`. Additional examples showing different functionalities of the robot arms `kinematics` class.

**Scripts**

* `reset_position.py`: A utility script to reset the robot arms to their initial positions.
* `pick_and_place.py`: Sample script that uses the `armbot` class to perform a simple, pre-programmed pick and place operation.
* `hello_arm.py`: Sample script that uses the `armbot` class to say *"hello"*.

---

##  Repository Structure

```sh
└── eva_armbot_control_class/
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
    ├── output
    │   └── image.png
    ├── samples
    │   ├── armbot
    │   │   ├── armbot class test 1.py
    │   │   ├── armbot class test 2.py
    │   │   ├── armbot class test 3.py
    │   │   ├── armbot class test 4.py
    │   │   ├── armbot class test 5.py
    │   │   ├── hello arm 2.py
    │   │   └── hello arm.py
    │   └── kinematics
    │       ├── sample code 1.py
    │       ├── sample code 2.py
    │       ├── sample code 3.py
    │       └── sample code 4.py
    ├── aravis.py
    ├── armbot.py
    ├── kinematics.py
    ├── hello_arm.py
    ├── pick_and_place.py
    ├── reset_position.py
    └── README.md
```

---

##  Modules

<details closed><summary>.</summary>

| File                                                                                                                       | Summary                         |
| ---                                                                                                                        | ---                             |
| [reset_position.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/reset_position.py)           | <code> </code> |
| [armbot class test 4.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/armbot class test 4.py) | <code> </code> |
| [armbot class test 1.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/armbot class test 1.py) | <code> </code> |
| [hello arm 2.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/hello arm 2.py)                 | <code> </code> |
| [sample code 3.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/sample code 3.py)             | <code> </code> |
| [armbot.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/armbot.py)                           | <code> </code> |
| [armbot class test 3.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/armbot class test 3.py) | <code> </code> |
| [kinematics.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/kinematics.py)                   | <code> </code> |
| [sample code 4.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/sample code 4.py)             | <code> </code> |
| [aravis.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/aravis.py)                           | <code> </code> |
| [sample code 2.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/sample code 2.py)             | <code> </code> |
| [armbot class test 2.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/armbot class test 2.py) | <code> </code> |
| [hello arm.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/hello arm.py)                     | <code> </code> |
| [armbot class test 5.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/armbot class test 5.py) | <code> </code> |
| [sample code 1.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/sample code 1.py)             | <code> </code> |

</details>

<details closed><summary>evasdk</summary>

| File                                                                                                                      | Summary                         |
| ---                                                                                                                       | ---                             |
| [eva_http_client.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/evasdk/eva_http_client.py) | <code> </code> |
| [eva_ws.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/evasdk/eva_ws.py)                   | <code> </code> |
| [EvaDiscoverer.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/evasdk/EvaDiscoverer.py)     | <code> </code> |
| [eva_locker.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/evasdk/eva_locker.py)           | <code> </code> |
| [eva_errors.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/evasdk/eva_errors.py)           | <code> </code> |
| [version.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/evasdk/version.py)                 | <code> </code> |
| [Eva.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/evasdk/Eva.py)                         | <code> </code> |
| [helpers.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/evasdk/helpers.py)                 | <code> </code> |
| [robot_state.py](https://github.com/Alexpascual28/eva_armbot_control_class.git/blob/master/evasdk/robot_state.py)         | <code> </code> |

</details>

---

#  Getting Started

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

##  Installation

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

##  Usage

To test the Armbot class with one of the test scripts, follow these steps:

Ensure your environment is set up with all necessary dependencies.
Navigate to the project directory.

**To run a test script**

Navigate to the sample directory where the file is located. Make sure the `kinematics`, `armbot`, `aravis` and `evasdk` packages are located in the same directory. Then run:

> ```console
> $ python3 <script name>.py
> ```

**To run the pick and place script**

This script executes a simple pick and place operation. You must first place the object to be picked, typically a cube, just in front of the robot arm, in the marked target position. Afterwards, in the main project directory, run the following:

> ```console
> $ python3 pick_and_place.py
> ```

**To run the hello arm script**

This script makes the robot arm wave its arm as if it is saying "hello". To run it, simply execute the following in the main project directory:

> ```console
> $ python3 hello_arm.py
> ```

**To run the reset position script**

In order to reset the arms pose to its original position, facing up, this script can be executed by running the following in the main project directory:

> ```console
> $ python3 reset_position.py
> ```

# Key Components

## Kinematics: Mathematical Foundations

`kinematics.py` defines the **Kinematics** class responsible for calculating the robot arm's movements. It uses mathematical models to determine how the arm should move its joints to reach a specific position.

### Inverse Kinematics and the Jacobian Matrix

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

## ArmBot

The **ArmBot** class, defined in `armbot.py`, using the kinematics class to control the robotic arm and integrates computer vision for object detection. To start the class you must first create an instance of it by providing the arm name as follows:

> ```python
> arm = Arm("evatrendylimashifterpt410")
> ```

Or the arm IP:

> ```python
> arm = Arm("144.32.152.105")
> ```

The arm and camera names and ips can be changed in the "initialize_arm()" and the "initialize_camera()" functions within the armbot class.

After an arm object has been instatiated, functions can be run using dot notation as usual. Key functions include:

### Main Functions of ArmBot Class

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

# Sample Scripts

## Pick and Place

This script, `pick_and_place.py`, is designed to control two robotic arms for a pick-and-place operation. Utilizing the `ArmBot` class, the script coordinates the movements of both arms to pick up objects from one location and place them in another. This example provides a clear demonstration of how to use the ArmBot class to manage multiple robotic arms simultaneously in a Python script.

### Main Functionality

The `main` function performs several steps:

1. **Initialize Two ArmBots**: Each `ArmBot` instance is created with a unique `hostname`, which corresponds to specific network details for each robotic arm. You can find more about network configurations on your internal network documentation or at the provided link in the script comments.

```python
# Details to connect to the robot arm, more info here: 
# https://wiki.york.ac.uk/display/TSS/Network+Details+and+Credentials+for+the+EVA+Arms+and+Network+Cameras
arm1 = ArmBot("evatrendylimashifterpt410");
arm2 = ArmBot("evacunningyorkartistpt410");
```

2. **Operational Sequence**:
   * Each robotic arm is homed using `home_robot()`, which positions the arm at a starting configuration.
   * The gripper on each arm is opened with `open_gripper()`, preparing to pick up an object.
   * Each arm moves to a specified location (`move_end_efector([0, -0.2, 0.18])`), closes its gripper (`close_gripper()`), and then returns to the home position.
   * The arms then move to place the objects at new locations, open the grippers to release the objects, and finally return to the home position where the grippers are closed.

3. **Error Handling**:
   * The operations are enclosed in a `try-except` block to handle interruptions like a `KeyboardInterrupt`. This is essential for stopping the script safely without leaving the robotic arms in an unstable state.
   * The `finally` block can include commands to reset the arms or ensure all resources are properly released, though these commands are commented out by default to emphasize the need for manual inspection post-error.

### Example Code
Here is a sample snippet from the script showing how one robotic arm is controlled:

```python
try:
   # Home robot and open the gripper
    arm1.home_robot();
    arm1.open_gripper();
    
    # Move to the target position and close the gripper
    arm1.move_end_efector([0, -0.2, 0.18]);
    arm1.close_gripper();
    arm1.home_robot();
    
    # Move to the drop-off location and open thye gripper to release the object
    arm1.move_end_efector([0.2, -0.2, 0.22]);
    arm1.open_gripper();
    
    # Home robot and close the gripper
    arm1.home_robot();
    arm1.close_gripper();
except KeyboardInterrupt:
    print("Exiting...")
```

## Hello Arm

This script, titled `hello_arm.py`, is designed to demonstrate basic operations of a robotic arm using the `ArmBot` class. With this script the arm says **"hello"** by waving back and forth between two positions. It involves moving the robotic arm through a sequence of positions while opening and closing the gripper, showcasing the arm's capabilities in a simple and repetitive manner.

### Main Functionality

The `main` function orchestrates a series of movements and gripper operations to demonstrate the robotic arm's functionality:

**Initialization**:

   * An instance of ArmBot is created for the robot arm, initialized with its specific network hostname.
   * A connection to the robotic arm is established based on this hostname.

**Operational Sequence**:

   1. The robotic arm is commanded to return to its *'home'* position to start from a known state.
   2. The arm then performs a series of movements to two different positions, alternating between opening and closing the gripper at each position.
   3. After performing these operations, the arm returns to the *initial* positions to repeat the gripper actions.
   4. Finally, the arm returns to the *'home'* position, ensuring it ends in a safe, default state.

**Error Handling**:

   * The script includes a `try-except` block to gracefully handle interruptions (like `KeyboardInterrupt`), allowing for safe termination of the program.
   * The `finally` block is prepared for resetting the arm if needed, though the reset command is commented out to emphasize manual inspection before automated restarts.

### Example Code

Here's a snippet showing the movement and gripper control sequence:

```python
try:
    arm.home_robot();
    
    pose1 = arm.move_end_efector([0.2, -0.2, 0.5]);
    arm.open_gripper();
    time.sleep(0.2)
    arm.close_gripper();
    time.sleep(0.2)
    
    pose2 = arm.move_end_efector([-0.2, -0.2, 0.5]);
    arm.open_gripper();
    time.sleep(0.2)
    arm.close_gripper();
    time.sleep(0.2)
    
    arm.set_joint_angles(pose1)
    arm.open_gripper();
    time.sleep(0.2)
    arm.close_gripper();
    time.sleep(0.2)
    
    arm.set_joint_angles(pose2)
    arm.open_gripper();
    time.sleep(0.2)
    arm.close_gripper();
    time.sleep(0.2)
    
    arm.home_robot();
except KeyboardInterrupt:
    print("Exiting...")
```

## Reset Position

This script, `reset_position.py`, is designed to reset two robotic arms to their default joint angles (facing up, so that the projector screen can be opened) using the `ArmBot` class. It serves as a basic utility script to ensure that the robotic arms are in a known, neutral state before starting any operations or after completing tasks to ensure safety and readiness for future operations.

### Main Functionality

The `main` function manages the resetting of joint angles for two robotic arms:

**Initialization**:

   * Two instances of `ArmBot` are created for each robot arm, initialized with their respective network hostnames.
   * A connection to each robotic arm is established based on these hostnames.

   ```python
   arm1 = ArmBot("evatrendylimashifterpt410");
   arm2 = ArmBot("evacunningyorkartistpt410");
   ```

**Operational Sequence**:

   1. Both robotic arms are instructed to set their joint angles to [0,0,0,0,0,0], which represents the default or neutral position for the arms.
   2. This position ensures that the arms are aligned straight without any bends or twists, which is often used as a safe configuration for starting or ending robotic operations.

**Error Handling**:

   * A `try-except` block captures `KeyboardInterrupt` to allow for safe termination of the program during manual interruptions.
   * The `finally` block is set up to potentially reset the arms if needed, though the reset command is commented out to stress the importance of manual inspection before automated resets.

### Example Code

Here's a simple code snippet illustrating how both robotic arms are reset to default positions:

```python
try:
    arm1.set_joint_angles([0,0,0,0,0,0]);
    arm2.set_joint_angles([0,0,0,0,0,0]);
except KeyboardInterrupt:
    print("Exiting...")
```

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
