# Chapter 1: The Robotic Nervous System (ROS 2)

**Coverage:** Weeks 3–5

Robots perceive, think, and act through software. For modern autonomous systems—humanoids, mobile robots, industrial manipulators—the “nervous system” that coordinates all sensing, planning, and control is **ROS 2** (Robot Operating System 2).
This chapter provides a detailed introduction to ROS 2, its architecture, tools, and workflows, enabling you to build multi-node, modular robotic systems.

---

# **1. What is ROS 2 and Why It Matters**

ROS 2 is an open-source robotics middleware designed for **real-time**, **reliable**, and **distributed** robot software systems.

## **1.1. Why ROS 2 Was Created**

ROS 1 dominated academic robotics but had limitations:

* No real-time guarantees
* Single-machine architecture
* Weak security
* Limited multi-robot capabilities
* Not suitable for embedded / safety-critical systems

ROS 2 was redesigned from the ground up to address these gaps using modern technologies such as **DDS (Data Distribution Service)**.

## **1.2. Key Advantages of ROS 2**

### **● Real-Time Communication (DDS)**

DDS provides deterministic publish–subscribe communication, essential for humanoid balance control, locomotion, and industrial robots.

### **● Multi-Robot Support**

ROS 2 allows robots to share topics over LAN/Wi-Fi without special bridging layers.

### **● Security (SROS 2)**

* Encrypted communication
* Authentication
* Access control policies

### **● Embedded Systems Support**

Compatible with microcontrollers (via **micro-ROS**) and high-performance embedded hardware.

### **● Cross-Platform**

Supports:

* Linux (primary)
* Windows
* macOS
* RTOS (via micro-ROS)

## **1.3. ROS 2 in Real-World Robots**

| Application         | How ROS 2 Helps                                   |
| ------------------- | ------------------------------------------------- |
| Autonomous vehicles | Sensor fusion, localization, planning             |
| Industrial robotics | Deterministic messaging & safety-critical control |
| Humanoid robots     | Multi-node architecture, simulation, URDF control |
| Drones              | Real-time communication, modular flight software  |

---

# **2. ROS 2 Architecture: Nodes, Topics, Services, Actions**

ROS 2 organizes robot software as a graph of distributed processes.

## **2.1. Nodes**

A **node** is a modular computation unit.
Examples:

* `camera_node` publishes images
* `controller_node` computes motor commands
* `localization_node` fuses sensor data

Nodes can run:

* On one machine
* Spread across multiple machines
* On microcontrollers

---

## **2.2. Topics**

Topics enable **asynchronous**, continuous data streaming.

* Many-to-many communication
* No request/response—messages just flow
* Examples:

  * `/image_raw` → camera images
  * `/joint_states` → robot joint positions
  * `/cmd_vel` → velocity commands

Each topic uses a **message type**, e.g.:

```bash
std_msgs/msg/String
sensor_msgs/msg/Image
geometry_msgs/msg/Twist
```

---

## **2.3. Services**

Services provide **synchronous request/response** communication.

Example use cases:

* Resetting a sensor
* Querying robot status
* Triggering a calibration routine

A service contains:

* Request
* Response

---

## **2.4. Actions**

Actions are used for **long-running tasks**:

* Navigation (“Go to this position”)
* Manipulation (“Pick up this object”)
* Humanoid control (“Execute walking gait sequence”)

An action includes:

* Goal
* Feedback
* Result
* Option to cancel/preempt

---

# **3. Installing ROS 2 Humble on Ubuntu 22.04**

ROS 2 Humble Hawksbill is the recommended LTS version.

## **3.1. System Setup**

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
export LANG=en_US.UTF-8
```

## **3.2. Add ROS 2 Repository**

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install curl -y
sudo curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | sudo apt-key add -
```

## **3.3. Install ROS 2 Humble (Desktop)**

```bash
sudo apt update
sudo apt install ros-humble-desktop
```

Alternatives:

* `ros-humble-ros-base` (minimal)
* Build from source (needed for bleeding-edge features)

## **3.4. Environment Setup**

Add ROS 2 to shell:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

# **4. Creating Your First ROS 2 Node in Python (rclpy)**

## **4.1. Create Workspace**

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## **4.2. Create a ROS 2 Package**

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_first_package
```

Directory structure:

```
my_first_package/
├── package.xml
├── setup.py
└── my_first_package
    └── __init__.py
```

---

## **4.3. Create a Publisher Node**

Create `publisher.py` inside `my_first_package`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = "Hello from ROS 2!"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

## **4.4. Create a Subscriber Node**

`subscriber.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f"I heard: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MinimalSubscriber())
    rclpy.shutdown()
```

---

## **4.5. Build and Run**

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

Run:

```bash
ros2 run my_first_package publisher
ros2 run my_first_package subscriber
```

---

# **5. Publisher–Subscriber Communication Patterns**

## **5.1. Message Types**

Use built-in message types:

* `std_msgs`
* `sensor_msgs`
* `geometry_msgs`
* `nav_msgs`

Or create custom messages via `.msg` files.

---

## **5.2. QoS (Quality of Service)**

DDS enables control of reliability:

* **Reliable**: No message loss
* **Best-effort**: Fast, but loss allowed
* **Keep-last**: Only store N messages
* **Keep-all**: Store entire history

Used for:

* Cameras
* LiDAR
* Control loops
* High-frequency sensors

---

## **5.3. Topic Inspection Tools**

```bash
ros2 topic list
ros2 topic echo /chatter
ros2 topic info /camera/image_raw
ros2 topic hz /cmd_vel
```

---

# **6. Building ROS 2 Packages**

## **6.1. Colcon Build System**

ROS 2 uses **colcon** for building workspaces:

```bash
colcon build --symlink-install
```

Workspaces contain packages in `src/`.

---

## **6.2. Understanding package.xml**

Defines metadata and dependencies:

Example:

```xml
<package format="3">
  <name>my_robot_pkg</name>
  <version>0.0.1</version>
  <maintainer email="example@gmail.com">Afaq</maintainer>
  <description>My robot software package.</description>

  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>rclpy</exec_depend>
</package>
```

---

## **6.3. Python Package Setup**

`setup.py` example:

```python
from setuptools import setup

package_name = 'my_robot_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'talker = my_robot_pkg.publisher:main',
            'listener = my_robot_pkg.subscriber:main',
        ],
    },
)
```

---

# **7. URDF for Humanoids**

URDF (Unified Robot Description Format) describes:

* Links
* Joints
* Geometry
* Inertial properties

## **7.1. Example Link Definition**

```xml
<link name="torso">
  <visual>
    <geometry>
      <box size="0.4 0.2 0.6"/>
    </geometry>
  </visual>
</link>
```

## **7.2. Joint Example**

```xml
<joint name="hip_joint" type="revolute">
  <parent link="torso"/>
  <child link="upper_leg"/>
  <origin xyz="0 0 -0.3"/>
  <axis xyz="1 0 0"/>
</joint>
```

## **7.3. Visualizing in RViz2**

```bash
ros2 launch robot_state_publisher display.launch.py
```

---

# **8. Launch Files and Parameter Management**

## **8.1. Python Launch Files**

Launch multiple nodes together:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot',
            executable='controller',
            parameters=[{'speed': 1.0}]
        ),
        Node(
            package='my_robot',
            executable='sensor_node'
        ),
    ])
```

Run:

```bash
ros2 launch my_robot robot.launch.py
```

---

## **8.2. Node Parameters**

Parameters are key–value configurations passed to nodes.

```bash
ros2 param list
ros2 param get /controller speed
ros2 param set /controller speed 2.0
```

---

# **9. Hands-On: Build a Multi-Node Robot Control System**

In this practical project, you will combine all concepts from this chapter.

## **9.1. System Overview**

A modular robot system with separate nodes:

* **Perception Node**
  Subscribes to camera / LiDAR

* **Planning Node**
  Computes path or gait sequences

* **Control Node**
  Sends velocity commands

## **9.2. Example Node Graph**

```
camera_node ───────────────► perception_node
perception_node ───────────► planning_node
planning_node ─────────────► control_node
control_node ──────────────► motors
```

---

## **9.3. Debugging Tools**

```bash
ros2 node list
ros2 node info /controller
ros2 topic hz /joint_states
ros2 run tf2_tools view_frames
```

---

# **End of Chapter Summary**

By the end of this chapter, you should be able to:

* Understand ROS 2 architecture
* Install and configure ROS 2
* Create Python nodes using `rclpy`
* Use topics, services, and actions
* Build ROS 2 packages
* Create URDF robot models
* Launch multi-node systems
* Design modular robot software architectures

