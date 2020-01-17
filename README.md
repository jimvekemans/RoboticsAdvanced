# Robotics Advanced 2019 - 2020

## Udemy Courses:
> https://www.udemy.com/course/ros-essentials/learn/ <br>
> https://www.udemy.com/course/ros-navigation/learn/ <br>
> https://www.udemy.com/course/pythoncv/learn/ <br>

# Goals
### Week 1

- can describe the difference between containers and virtual machines
- can explain the difference between containers and images
- can explain what Docker is
- can describe what Registries are
- can install and configure Docker / Nvidia-Docker
- can create docker containers from existing images
- can explain Docker volumes
- can create, remove, use Docker volumes
- can list, start, attach, stop, remove containers from existing images
- can inspect running containers
- can list, remove Docker container images
- can install Portainer
- can inspect and manage Docker containers using Portainer
- can read, write and build container images using a Dockerfile
- can multiplex a terminal using Tmux
- can start a Turtlebot2 simulation and control it with the keyboard
- can inspect Turtlebot2 topics and messages
- can write Python ROS code to move the Turtlebot2
- can use OpenCV to analyse the camera feed of the Turtlebot2

### Week 2

- can explain what a container is
- can explain the benefits of containers
- can explain all docker stages
- understands why decouple applications are good
- can explain the link between decoupled applications and multiple containers
- can write and use docker-compose files to create multi container ROS architectures

### Week 3

- can explain what a drone is in context of robotics
- can explain the abbreviations UAV and RPAS
- can layout an taxonomy for UAVs/Drones
- can describe why drones became popular during the 2010s
- can categorize UAVs in to the corresponding types: fixed wing, multirotor/multicopter, quadcopter, hexacopter, hybrid
- can list the different drone usages and provide at least one example per usage
- can explain why drone safety and legislation is a necessity.
- can describe the difference between aerostatic and aerodyne aeroplanes
- can explain the four different forces that exist during flight
- can explain the Bernoulli’s principle
- can describe pitch, roll and yaw and draw a figure to depict the different axes
- understands blade rotation, torque and tail rotor thrust in context of helicopters
- understands pitch, roll and yaw in context of helicopters
- can explain lift, hoovering in context of helicopters
- can explain collective pitch rotors helicopters and UAVs
- can explain fixed pitch rotors in context of UAVs
- can explain the different electronic components of an UAV:  autopilot, motors, ground station, GPS antenna, radio telemetry, remote  control, battery, buzzer, ESC (electronic speed controllers.
- can explain the modes of motion of a quadcopter and the direction and speed of its rotors
- can explain the magnitudes of lift
- can start the Parrot AR.Drone 2.0 simulation world
- understands the different topics provided by the AR.Drone 2.0 ROS driver
- can write ROS code to control the UAV
- can write OpenCV code to analyze the UAVs camera feed.

### Week 4

- can describe in own words the PX4 project and its software
- can describe in own words MAVLINK, QGroundControl, MAVSDK
- can control a MAVLINK UAV using ROS
- can describe in own words companion computer
- can draw and completely explain Smart UAV architecture

### Week 5

- can describe in own words the SLAM problem including its components (mapping and localization)
- can explain the link between SLAM and the “chicken-egg” problem
- can describe and explain the accumulation of uncertainty in context of sensors
- can explain long-term position estimation without a priori information
- can describe and explain the accumulation of uncertainty in context of SLAM
- knows that there are multiple estimation methods, for example Kalman and Particle (How they work is out of scope!)
- can divide localization methods starting from the position acquisition.
- can explain in own words relative and absolute positioning and the difference between them
- can explain odometry and intertial navigation in context of absolute position measurement
- can explain beacons and visions systems in context of absolute position measurement
- can describe odometry and give an example
- can explain dead reckoning
- can explain intertial navigation including the term IMU
- can pinpoint the most significant error source of rotary encoders
- can explain drive and other issues with IMUs
- can explain the link between LIDAR, optical sensors, WiFi, GPS and absolute positioning measurement
- can explain why WiFi isn’t a good sensor in the context of SLAM
- can list at least two issues with GPS sensors
- can describe in own words the term Visual Odometry (VO).
- can explain the difference between monocular and stereo cameras in context of VO
- can explain the difference between feature based and direct method in context of VO
- can describe visual inertial odometry in own words
- can describe (a 6 step) generic VO algorithm
- can explain egomotion
- can explain the benefit of multi sensor fusion and can draw an example to support the explanation.
- can explain the basic idea of Markov localization and draw a simple figure to support the explanation.
- can list at least two SLAM applications
- can describe the difference between interoceptive and exteroceptive  sensors and there link with relative or absolute position measurements
- can explain how acoustic sensors measure distance; where they are applicable; and their shortcomings
- can explain how Laser rangefinders measure distance; where they are applicable; and their shortcomings
- can describe how monocular cameras measure distance
- can explain the unknown scale factor issue including dimensionless maps
- can describe a solution for the unknown scale factor issue.
- can describe in own words how stereo cameras work
- can explain the term FPGA
- can describe in own words how RGB-D cameras work, including the  difference between structured light and time-of-flight and their  shortcomings
- can explain the difference between feature maps and occupancy grids
- can draw a simple architecture of a SLAM system divided in frond-end and back-end
- can explain the difference between sparse and dense Visual SLAM methods
- can explain the difference between feature-based and direct Visual SLAM methods
- can describe feature extraction and feature matching in context of Visual SLAM methods
- can explain loop closure in context of SLAM
- can explain why back-end optimization is needed and the difference between camera pose optimization and bundle adjustment
- can run a gmapping SLAM with the TurtleBot2

### Week 6

- can describe the term multi agent systems
- can explain Distributed artificial intelligence; Parallel AI Distributed problem solving and multi-agent-systems (MAS)
- can describe an agent in the context of MAS
- can draw the typical building blocks of an autonomous agent and describe it
- can explain the link between agent technology and MAS
- can list at least two MAS benefits
- can list at least two MAS critical challenges
- can explain the difference between homogeneous structure and heterogeneous structure in contact of MAS
- can explain the difference between hierarchical organization,  holonic agent organization, coalitions teams and  in contact of MAS and  can draw a figure per type
- can describe the difference between local communication and  backboards in context of MAS using self drawn figures to support the  explanation
- can describe blackboards in a multi computer setup using self drawn figure to support the explanation
- can describe hierarchical state machine
- can create a simple hierarchical state machine using the SMACH library
- can describe active learning, reactive learning and learning based on consequence in context of MAS
- can describe the term Swarms
- can draw a simple representation of the ROS1 and ROS2 architecture to depict the differences
- can explain the differences between the ROS1 and ROS2 architectures
- can draw a figure depicting  the ROS2, DDS, UDP/IP and Ethernet stack and a OSI 7 layer model
- can explain the benefit of DDS using the OSI 7 layer model
- can create a simple ROS2 robot project using publishers and subscribers
- can explain the ros_brige to connect ROS1 with ROS2

### Week 7

- can explain why computer vision is hard
- can, by using an example, explain why having no prior knowledge could be a problem for Neural Networks and how to solve it.
- can explain the need for edge cases in an image dataset
- can give an example which can occur when using a dataset build out of uniform samples
- can explain the benefit of using simulations in the context of RL
- can describe why simulations are hard in context of RL and Robotics
- can explain how Tesla is able to accumulate so much annotated (training) data
- can describe what reinforcement learning (RL) is, including: action, observation and reward.
- can list and describe four RL examples
- can give an example of a reward shaping side effect (alignment problem or unintended behavior)

### Week 8

- Project

# Project

### Requirements

The requirements are very simple and give as much creative freedom as possible. The requirement: incorporate the course topics as much as possible in the project.

#### Topics:

- (Multi) Container ROS architectures
- Rapid Drone Software Prototyping
- Simultaneous localization and mapping
- Multi Agent Systems
- ROS2
- Reinforcement learning (RL)

**Remark: it's no problem if some topics are much more implemented than others**

For example: A Turtlebot2 soccer team could have a focus on Multi  container ROS architectures, Simultaneous localization and mapping,  Multi Agent Systems, ROS2 and Reinforcement learning (RL). But only very small or even no Rapid Drone Software Prototyping feature.

### Evaluation

At the end of the semester there will be an oral exam. This will  consist out of a deep dive into your group project and an examination of the weekly goals (Week 1 - Week 7).

<hr>
## Objective:<br>

### - Make 2 turtlebots play soccer, a drone functions as referee
## Subtasks:
### - turtlebots should detect the ball
### - turtlebots should move towards the ball
### - turtlebots should use SLAM to create a map of the field
### - turtlebots should try to move the ball in the direction of the other goal
    - colour the goals differently to make the right goal obvious to turtlebot
### - drone should detect ball
### - drone should detect all turtlebots
### - drone should hover above the ball 

#### Used sources:
- https://answers.ros.org/question/41073/multiple-turtlebots-in-gazebo/
- https://answers.ros.org/question/41433/multiple-robots-simulation-and-navigation/
