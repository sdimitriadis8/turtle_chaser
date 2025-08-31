## 🔧 Build Instructions

1. Navigate to your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
```

2. Clone this repository:
```bash
git clone https://github.com/<your-username>/turtle_chaser.git
```


3. Build the workspace:
```bash
cd ~/ros2_ws
colcon build --symlink-install
```


4. Source the workspace:
```bash
source install/setup.bash
```
---

## 🚀 Usage

1. Run the turtle chaser app

This launches turtlesim, spawns turtles, and runs the chasing logic:

```bash
ros2 launch my_robot_bringup catch_all_turtles_app.xml
```

2. Run the number publisher app

Publishes numbers and processes them with different nodes:

```bash
ros2 launch my_robot_bringup number_app.launch.xml
```

3. Run the robot news app

Demonstrates a publisher/subscriber pattern with "news stations" and "smartphones":

```bash
ros2 launch my_robot_bringup robot_news_app.launch.xml
```
---

## 📁 Project structure

---
turtle_chaser/
├── .gitignore
├── LICENSE
├── my_cpp_pkg/ # C++ ROS 2 nodes
│ ├── src/ # Node implementations (publishers, servers, etc.)
│ ├── include/ # C++ headers
│ ├── CMakeLists.txt
│ └── package.xml
│
├── my_py_pkg/ # Python ROS 2 nodes
│ ├── my_py_pkg/ # Python scripts (publishers, clients, turtlesim control, etc.)
│ ├── resource/
│ ├── test/ # Linting & style tests
│ ├── setup.py
│ ├── setup.cfg
│ └── package.xml
│
├── my_robot_bringup/ # Configs & launch files
│ ├── config/ # YAML configs
│ ├── launch/ # Launch files for apps
│ ├── CMakeLists.txt
│ └── package.xml
│
├── my_robot_interfaces/ # Custom ROS 2 messages & services
│   ├── msg/ # HardwareStatus, LedPanelState, Turtle, TurtleArray
│   ├── srv/ # CatchTurtle, ComputeRectangleArea, SetLed
│   ├── CMakeLists.txt
│   └── package.xml
└── README.md


## 📜 Features

✅ C++ & Python nodes (publishers, subscribers, services, clients)

✅ Custom ROS 2 interfaces (.msg and .srv)

✅ Multiple launch files to manage apps

✅ Demonstrates turtlesim control, spawning, and chasing logic

---

## License

This project is licensed under the MIT License

---

## Author: Sokratis Dimitriadis
