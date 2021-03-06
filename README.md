
# Kinematics Pick & Place Project
Robotics Nano Degree

[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)

[//]: # (Written by Nick Hortovanyi Oct 30th 2017)

---
![Gazebo Kuka Arm Simulator Output](https://github.com/hortovanyi/RoboND-Kinematics-Project/blob/master/output/Gazebo-KukuArm.png?raw=true)

The goal of this project was to provide kinematic joint trajectories to control the Kuka serial arm robot manipulator in  a gazebo simulation.


## Kinematic Analysis

#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

The following is a diagram that highlighting the 6 joints (with a reference frame for the gripper/end effector) and the associated links (starting at 0) between the joints.
![Kuka KR210 Kinematics - Joints and Links](https://github.com/hortovanyi/RoboND-Kinematics-Project/blob/master/output/Kuka%20KR210%20Kinematics.png?raw=true)

There is a Unified Robot Description Format (URDF) file [kr210.urdf.xacro](https://github.com/hortovanyi/RoboND-Kinematics-Project/blob/master/kuka_arm/urdf/kr210.urdf.xacro) which contains the details of the robot arm. To visual inspect this we use [RViz](http://wiki.ros.org/rviz) (which is a 3D visualiser).

![Forward Kinematics Rviz](https://github.com/hortovanyi/RoboND-Kinematics-Project/blob/master/output/Forward_Kinematics.png?raw=true)

Through visual inspection of the URDF & RViz application the following [DH Parameter](https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters) table was constructed.

Links | alpha(i-1) | a(i-1) | d(i) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | |
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + O2
2->3 | 0 | 1.25 | 0 | |
3->4 | - pi/2 | -0.054 | 1.50 | |
4->5 | pi/2 | 0 | 0 | |
5->6 | - pi/2 | 0 | 0 | |
6->EE | 0 | 0 | 0 | |

Alpha(i-1) = the twist angle, a(i-1) = the distance, d(i) = the link length and theta(i) (corresponds to Oi on the diagram) for joint angles.

For link 1->2 a 90 degree angle translation was added otherwise all other joints are at 0 degree angle in the URDF.

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

##### Transformation Matrices

A template matrix, sample code follows was used where the parameters were replaced from the DH parameter table to create the 7 individual transformations.

```
Matrix([
[            cos(q),            -sin(q),            0,              a],
[sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
[sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
[                 0,                  0,            0,              1]
])
```

###### T0->1
```
⎡cos(q₁)  -sin(q₁)  0   0  ⎤
⎢sin(q₁)  cos(q₁)   0   0  ⎥
⎢   0        0      1  0.75⎥
⎣   0        0      0   1  ⎦
```
###### T1->2
```
⎡cos(q₂ - 0.5⋅π)   -sin(q₂ - 0.5⋅π)  0  0.35⎤
⎢       0                 0          1   0  ⎥
⎢-sin(q₂ - 0.5⋅π)  -cos(q₂ - 0.5⋅π)  0   0  ⎥
⎣       0                 0          0   1  ⎦
```
###### T2->3
```
⎡cos(q₃)  -sin(q₃)  0  1.25⎤
⎢sin(q₃)  cos(q₃)   0   0  ⎥
⎢   0        0      1   0  ⎥
⎣   0        0      0   1  ⎦
```
###### T3->4
```
⎡cos(q₄)   -sin(q₄)  0  -0.054⎤
⎢   0         0      1   1.5  ⎥
⎢-sin(q₄)  -cos(q₄)  0    0   ⎥
⎣   0         0      0    1   ⎦
```
###### T4->5
```
⎡cos(q₅)  -sin(q₅)  0   0⎤
⎢   0        0      -1  0⎥
⎢sin(q₅)  cos(q₅)   0   0⎥
⎣   0        0      0   1⎦
```
###### T5->6
```
⎡cos(q₆)   -sin(q₆)  0  0⎤
⎢   0         0      1  0⎥
⎢-sin(q₆)  -cos(q₆)  0  0⎥
⎣   0         0      0  1⎦
```
###### T6->G or EE (no rotation)
```
⎡1  0  0    0  ⎤
⎢0  1  0    0  ⎥
⎢0  0  1  0.303⎥
⎣0  0  0    1  ⎦
```

##### Homogeneous Transformation matrix from base_link to Gripper


```
T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G
T_total = T0_G * R_corr
```
Note: R_corr is a 180 degree rotation around the Z axis followed by a -90 degree rotation around the Y axis

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

##### Inverse Position
The spherical wrist involves joints 4,5 and 6. The position of the wrist centre is governed by the first three joints.

We can derive the wrist centre by using the complete transformation matrix. Symbolically the homogeneous transform follows

```
⎡lx  mx  nx  px⎤
⎢ly  my  ny  py⎥
⎢lz  mz  nz  pz⎥
⎣0   0   0   1 ⎦
```
where l,m and n are orthonormal vectors representing the end-effector orientation along X,Y,Z axes of local coordinates.

```
wcx = px - (d6 + l) * nx
wcy = py - (d6 + l) * ny
wcz = pz - (d6 + l) * nz
```
where l is the end effort length and d6 = 0 (link 6 length).

To calculate nx, ny and nz, rotation matrices are created with error correction.

```
ROT_x = Matrix([[1, 0,0],
                [0, cos(r), -sin(r)],
                [0, sin(r), cos(r)]
                ])  # ROLL
ROT_y = Matrix([[cos(p), 0, sin(p)],
                [0, 1, 0],
                [-sin(p), 0, cos(p)]
                ])  # PITCH
ROT_z = Matrix([[cos(y), -sin(y), 0],
                [sin(y), cos(y), 0],
                [0, 0, 1]
                ])  # YAW

Rot_Error = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))
ROT_EE = ROT_z * ROT_y * ROT_x * Rot_Error
Rrpy = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

nx = Rrpy[0,2]
ny = Rrpy[1,2]
nz = Rrpy[2,2]
```
Where euler roll, pitch and yaw are extracted from the quaternion for the end effector pose and thus Rrpy = Homogeneous RPY rotation between base and gripper.

`theta1 = atan2(wcy, wcx)` and is clipped to +- 185 degrees.

![TTheta 2 and Theta 3 Angles Diagram](https://github.com/hortovanyi/RoboND-Kinematics-Project/blob/master/output/l21-l-inverse-kinematics-new-design-fixed.png?raw=true)

where

```
side_a = 1.501 #(d4)
side_b = sqrt(pow((sqrt(wcx * wcx + wcy * wcy) - 0.35), 2) + pow((wcz - 0.75), 2))
side_c = 1.25 #(a2)
```



`angle_a`, `angle_b` and `angle_c` are derived using [SSS triangle rules](https://www.mathsisfun.com/algebra/trig-solving-sss-triangles.html).

```
Theta2 = pi/2 - angle_a - atan2(wcz - 0.75, sqrt(wcx * wcx + wcy * wcy) - 0.35)
Theta3 = pi/2 - (angle_b + 0.036)  # 0.036 accounts for sag in link4 of -0.054 m`
```

Theta 2 is clipped to -45 and +85 degrees. Theta 3 is clipped to -210 and +155-90 degrees.

[//]: # (Written by Nick Hortovanyi Oct 30th 2017)

##### Inverse Orientation

Given that

```
R0_6 = R0_1*R1_2*R2_3*R3_4*R4_5*R5_6
and
R0_6 = Rrpy
```
we can precalculate rotations R0_3 (with theta 1 to 3 substituted) by extracting the rotation matrix from the transformation matrices

```
R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
```

leading to when we divide both sides by `R0_3` to

```
R3_6 = inv(R0_3) * Rrpy
```

Theta 4,5,6 are derived from R3_6 using Euler Angles from Rotation Matrix.

```
theta4 = atan2(R3_6[2,2], -R3_6[0,2])
theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
theta6 = atan2(-R3_6[1,2], R3_6[1,0])
```
Theta 4 & 6 is clipped +-350 degrees. Theta 5 to +-125 degrees.
All clipping values were extracted for lower and upper limit sassociated with the [joints from the kr210.urdf](https://github.com/hortovanyi/RoboND-Kinematics-Project/blob/cb21cc74037d9e51ee6fe7c50a39acdeedbd387d/kuka_arm/urdf/kr210.urdf.xacro#L316) file.

## Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.

I decided initially in this project to not refactor the code into using classes and abstracting away detail. This was mainly due to the mathematical nature of it and the use of [sympy](http://www.sympy.org/en/index.html). In the [FK_debug.py](https://github.com/hortovanyi/RoboND-Kinematics-Project/blob/master/FK_debug.py) script for the forward kinematics I added debug logging statements to time how long calculations were taking. The `simplify(<fomula>)` function was too slow. Removing them dramatically improved performance.

Whilst I enjoyed learning about sympy, I felt that numpy may have been faster and maybe a better option, if I was start from scratch on this project.

In addition, if I was to redo this project, I'd spend more time creating a class structure/overloaded methods to implement the mathematical rules with associated unit tests. Whilst the code looks concise, for someone learning about this subject, there is no easy way to confirm that the correct maths and rules have been applied, except via trial and error.

When running the simulator the hand often flip/flopped (ie it hasn't travelled the shortest distance, but the longest to get to the desired angle). To overcome this to some degree I found that the course note recommended approach of `R3_6 = R0_3.inv("LU") * Rrpy` should be changed to `R3_6 = R0_3.transpose() * Rrpy`. It improved results dramatically but there were still some instances were it flipped. I thought about smoothing angles for the wrist joints but didn't implement.

Occasionally I also noticed some collisions when joints moved, so I clipped the angles to the limit values in the kr210.urdf.

A video follows of a successful run. I ran the simulation >10 times with one failed attempt before submitting the project.

[![YouTube Video of project output x 4](http://img.youtube.com/vi/tg0pc-ngPQk/0.jpg)](https://www.youtube.com/watch?v=tg0pc-ngPQk)

[YouTube Video of project output x 4](https://www.youtube.com/watch?v=tg0pc-ngPQk)


---
[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)
# Robotic arm - Pick & Place project

Make sure you are using robo-nd VM or have Ubuntu+ROS installed locally.

### One time Gazebo setup step:
Check the version of gazebo installed on your system using a terminal:
```sh
$ gazebo --version
```
To run projects from this repository you need version 7.7.0+
If your gazebo version is not 7.7.0+, perform the update as follows:
```sh
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install gazebo7
```

Once again check if the correct version was installed:
```sh
$ gazebo --version
```
### For the rest of this setup, catkin_ws is the name of active ROS Workspace, if your workspace name is different, change the commands accordingly

If you do not have an active ROS workspace, you can create one by:
```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Now that you have a workspace, clone or download this repo into the **src** directory of your workspace:
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/udacity/RoboND-Kinematics-Project.git
```

Now from a terminal window:

```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ sudo chmod +x target_spawn.py
$ sudo chmod +x IK_server.py
$ sudo chmod +x safe_spawner.sh
```
Build the project:
```sh
$ cd ~/catkin_ws
$ catkin_make
```

Add following to your .bashrc file
```
export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/models

source ~/catkin_ws/devel/setup.bash
```

For demo mode make sure the **demo** flag is set to _"true"_ in `inverse_kinematics.launch` file under /RoboND-Kinematics-Project/kuka_arm/launch

In addition, you can also control the spawn location of the target object in the shelf. To do this, modify the **spawn_location** argument in `target_description.launch` file under /RoboND-Kinematics-Project/kuka_arm/launch. 0-9 are valid values for spawn_location with 0 being random mode.

You can launch the project by
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ ./safe_spawner.sh
```

If you are running in demo mode, this is all you need. To run your own Inverse Kinematics code change the **demo** flag described above to _"false"_ and run your code (once the project has successfully loaded) by:
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ rosrun kuka_arm IK_server.py
```
Once Gazebo and rviz are up and running, make sure you see following in the gazebo world:

	- Robot

	- Shelf

	- Blue cylindrical target in one of the shelves

	- Dropbox right next to the robot


If any of these items are missing, report as an issue.

Once all these items are confirmed, open rviz window, hit Next button.

To view the complete demo keep hitting Next after previous action is completed successfully.

Since debugging is enabled, you should be able to see diagnostic output on various terminals that have popped up.

The demo ends when the robot arm reaches at the top of the drop location.

There is no loopback implemented yet, so you need to close all the terminal windows in order to restart.

In case the demo fails, close all three terminal windows and rerun the script.
