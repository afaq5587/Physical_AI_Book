# Chapter 2: The Digital Twin (Gazebo & Unity)

**Coverage:** Weeks 6–7

A “Digital Twin” is a high-fidelity virtual replica of a robot and its environment. It allows developers to test algorithms, validate robot behavior, and simulate complex interactions long before deploying them to real hardware.
This chapter explores physics simulation for robotics, with a focus on **Gazebo**, **Unity**, and ROS 2 integration.

---

# **1. Introduction to Physics Simulation**

Simulation allows developers to iterate quickly, safely, and cost-effectively before shifting to real robots.

## **1.1. Why Simulation Matters in Robotics**

Robotics development is expensive and risky. Simulation provides:

* **Cost Reduction**: No need to repeatedly replace broken hardware.
* **Safety**: Dangerous experiments (falls, collisions, high-speed movement) conducted virtually.
* **Rapid Prototyping**: Faster development cycles for testing algorithms.
* **Scalability**: Run thousands of tests automatically (CI/CD simulation pipelines).
* **Environment Variation**: Try different lighting conditions, terrains, dynamic obstacles.

Modern humanoid robotics relies heavily on digital twins for locomotion research, balance control, and manipulation.

---

## **1.2. Core Concepts of Physics Engines**

Robotics simulators use dedicated physics engines that approximate real-world mechanics.

### **● Rigid Body Dynamics**

Simulates the motion of bodies assuming they do not deform.
Robots are modeled as:

* **Links** (rigid bodies)
* Connected with **joints** (constraints)

### **● Joint Constraints**

Control the robot’s kinematic structure:

* Revolute
* Prismatic
* Fixed
* Continuous
* Spherical

### **● Collision Detection**

Determines when bodies intersect or make contact using simplified shapes:

* Box
* Sphere
* Cylinder
* Mesh

### **● Solvers**

Physics engines use solvers for stability and realism:

* DART (high fidelity)
* ODE (stable, widely used)
* Bullet (fast, widely adopted in gaming)
* Simbody (accurate biomechanics)

---

## **1.3. Popular Robotics Simulators**

| Simulator                           | Strengths                                   | Use Cases                                     |
| ----------------------------------- | ------------------------------------------- | --------------------------------------------- |
| **Gazebo (Ignition/Gazebo Garden)** | High-fidelity physics, native ROS 2 support | Humanoids, mobile robots, autonomous vehicles |
| **Unity3D**                         | Photorealistic rendering & VR               | HRI, perception training                      |
| **NVIDIA Isaac Sim**                | GPU-accelerated, high realism               | Manipulation, AI learning                     |
| **Webots**                          | Simple, stable                              | Education, research                           |
| **MuJoCo**                          | Smooth physics                              | Reinforcement learning                        |

For this course, Gazebo + Unity provide the ideal balance of realism and speed.

---

# **2. Setting Up Gazebo Simulation Environment**

Gazebo is tightly integrated with ROS 2 via the **ros_gz** bridge.

---

## **2.1. Installing Gazebo (Gazebo Garden)**

```bash
sudo apt update
sudo apt install gazebo
sudo apt install ros-humble-ros-gz
```

Verify:

```bash
gazebo
```

---

## **2.2. ROS 2 Integration: ros_gz**

`ros_gz` enables communication between ROS 2 messages and Gazebo Transport messages.

Example bridge command:

```bash
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
```

---

## **2.3. Understanding Gazebo Components**

### **● Worlds**

Complete simulation environments containing:

* Ground plane
* Lights
* Models
* Physics settings

World files use the **SDF (.sdf)** format.

---

### **● Models**

A model can be:

* A robot
* A sensor
* A static object (wall, table, human figure)
* A dynamic object (box, ball, door)

Models include:

* Links
* Joints
* Collision geometry
* Inertial properties
* Visual meshes
* Plugins

---

### **● Plugins**

Custom code that extends robot or world functionality:

* Motors & controllers
* Sensors (LIDAR, IMU)
* Physics modifiers
* ROS 2 communication plugins

Plugins are typically written in C++.

---

## **2.4. Basic Gazebo Controls**

You can:

* Move camera (middle mouse + drag)
* Rotate camera (right mouse + drag)
* Spawn models from the library
* Pause/resume physics
* Inspect models using the GUI

---

# **3. SDF and URDF Robot Description Formats**

## **3.1. URDF (Unified Robot Description Format)**

Used primarily by **ROS 2** for:

* Kinematics
* Visualization (RViz)
* Joint limits
* Collision geometry

URDF = great for robots, but **limited** for rich simulation.

---

## **3.2. SDF (Simulation Description Format)**

SDF is more expressive:

* Complex environments
* Sensor parameters
* Light sources
* Water + atmosphere
* Multi-robot scenes
* Physics configurations

SDF is the native format used by Gazebo.

---

## **3.3. When to Use Which?**

| Feature                    | URDF | SDF |
| -------------------------- | ---- | --- |
| Robot kinematics           | ✔️   | ✔️  |
| Advanced sensor simulation | ❌    | ✔️  |
| Environment/worlds         | ❌    | ✔️  |
| Simple humanoids           | ✔️   | ✔️  |
| Complex multi-body physics | ❌    | ✔️  |

---

## **3.4. Converting URDF → SDF**

Gazebo can convert URDF automatically:

```bash
gz sdf -p your_robot.urdf > your_robot.sdf
```

---

## **3.5. Using Xacro for Modular URDF**

Xacro allows URDF files to be written as templates:

```xml
<xacro:property name="leg_length" value="0.5"/>
<link name="leg_link">
  <visual>
    <geometry>
      <cylinder length="${leg_length}" radius="0.05"/>
    </geometry>
  </visual>
</link>
```

Run:

```bash
xacro humanoid.xacro > humanoid.urdf
```

---

# **4. Physics Parameters: Gravity, Friction, Collisions**

## **4.1. Physics Engines in Gazebo**

Choose engine in SDF:

```xml
<physics type="ode">
```

### Engines:

* **ODE** – stable, default
* **Bullet** – good for contact-heavy scenes
* **DART** – best for humanoids
* **Simbody** – highly accurate

---

## **4.2. Key Physics Properties**

### **● Gravity**

```xml
<gravity>0 0 -9.81</gravity>
```

### **● Inertia**

```xml
<inertial>
  <mass>12.0</mass>
  <inertia>
    <ixx>1.2</ixx>
    <iyy>1.3</iyy>
  </inertia>
</inertial>
```

Correct inertia is critical for humanoid balance.

---

### **● Friction**

```xml
<surface>
  <friction>
    <ode>
      <mu>1.0</mu>
      <mu2>1.0</mu2>
    </ode>
  </friction>
</surface>
```

### **● Collisions**

```xml
<collision name="foot_collision">
  <geometry>
    <box><size>0.2 0.1 0.05</size></box>
  </geometry>
</collision>
```

---

# **5. Simulating Sensors: LIDAR, Cameras, IMUs**

## **5.1. Adding Sensors in SDF**

### **LIDAR Example**

```xml
<sensor name="lidar" type="gpu_lidar">
  <update_rate>20</update_rate>
  <topic>/scan</topic>
</sensor>
```

---

### **Depth Camera**

```xml
<sensor name="depth_camera" type="depth_camera">
  <topic>/depth</topic>
</sensor>
```

---

### **IMU**

```xml
<sensor name="imu" type="imu">
  <topic>/imu</topic>
</sensor>
```

---

## **5.2. ROS 2 Sensor Bridging**

Example:

```bash
ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan
```

---

## **5.3. Noise Simulation**

Add sensor noise parameters:

```xml
<noise>
  <type>gaussian</type>
  <stddev>0.01</stddev>
</noise>
```

Essential for testing algorithms under realistic conditions.

---

# **6. Creating Realistic Simulation Environments**

## **6.1. Importing CAD Models**

Gazebo supports:

* `.dae`
* `.stl`
* `.obj`

Place them inside:

```
~/.gazebo/models/
```

---

## **6.2. Adding Textures & Lighting**

```xml
<visual>
  <material>
    <script>
      <uri>file://media/materials/scripts</uri>
      <name>RustyMetal</name>
    </script>
  </material>
</visual>
```

---

## **6.3. Dynamic Objects**

Examples:

* Doors
* Tables
* Moving obstacles
* Human figures

Dynamic objects improve realism for HRI and manipulation tasks.

---

## **6.4. Complex Scenes**

You can design:

* Lab rooms
* Warehouses
* Outdoor terrains
* Urban environments
* Emergency rescue simulations

---

# **7. Unity for High-Fidelity Rendering**

Unity provides **photorealistic graphics** and VR support.

---

## **7.1. Why Use Unity for Robotics?**

* High-quality rendering
* Realistic lighting and shaders
* Integration with ML tools
* VR/AR capabilities
* Custom UI for human-robot interaction experiments

---

## **7.2. ROS 2 Integration with Unity**

Unity communicates with ROS 2 using:

* **Unity Robotics Hub**
* **ROS–TCP–Connector**

---

### **Steps:**

1. Install Unity (2021+ recommended)
2. Add the ROS-TCP-Connector package
3. Import URDF through **URDF Importer**
4. Publish/subscribe to ROS 2 topics

---

## **7.3. Example: Subscribe to a Camera Feed**

Unity C#:

```csharp
var subscriber = ROSConnection.instance.Subscribe<ImageMsg>(
    "/camera/image_raw",
    OnImageReceived);
```

Unity is particularly useful for perception, HRI, and visual testing.

---

# **8. Human–Robot Interaction (HRI) Scenarios**

## **8.1. Simulating Humans**

Gazebo + Unity can load:

* Animated human models
* Skeleton motion capture data
* Behavior scripts (walking, waving, sitting)

---

## **8.2. Collaborative Robotics Scenarios**

Simulate:

* Shared workspace
* Human handovers
* Safe stop behaviors
* Proximity-based speed control

---

## **8.3. User Experience Evaluation**

Unity allows:

* Virtual reality user studies
* Interaction prototypes
* Visual feedback systems

---

# **9. Hands-On Project: Build a Complete Simulation World**

You will develop a full simulation world with a humanoid robot.

---

## **9.1. Steps**

### **1. Create a new Gazebo world**

Add:

* Sky
* Lights
* Ground plane

### **2. Import humanoid URDF/SDF**

Use:

```bash
ros2 launch robot_state_publisher display.launch.py
```

### **3. Add sensors**

* LIDAR
* RGB camera
* Depth camera
* IMU

### **4. Add objects**

* Boxes
* Ramps
* Obstacles

### **5. Test ROS 2 topics**

```bash
ros2 topic list
ros2 topic echo /scan
ros2 topic echo /imu
```

### **6. Implement a simple task**

Examples:

* Walk to a target
* Pick and place
* Navigate around obstacles

---

# **End of Chapter Summary**

By the end of this chapter you will be able to:

* Understand physics simulation concepts
* Use Gazebo and Unity as digital twin platforms
* Create URDF/SDF robot models
* Integrate sensors and environments
* Build realistic world models
* Connect simulation data with ROS 2
* Run full humanoid simulation scenarios



