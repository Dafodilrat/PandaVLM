# 🤖 Integrating Vision-Language Models for Autonomous Robotic Manipulation

[![ROS 2](https://img.shields.io/badge/ROS-2-blue)](https://docs.ros.org/en/foxy/index.html)
[![MoveIt 2](https://img.shields.io/badge/MoveIt-2-green)](https://moveit.picknik.ai/)
[![OpenAI](https://img.shields.io/badge/OpenAI-GPT--4V-red)](https://openai.com/)

## 📖 Overview
This project explores the use of **Vision-Language Models (VLMs)** for autonomous robotic manipulation, enabling a robotic arm to understand and execute **natural language commands**.  

The system combines:
- **Language understanding** (LLMs / VLMs)  
- **Visual perception** (RGB + depth sensing)  
- **Coordinate transformation** (camera → robot base)  
- **Motion planning & control** (MoveIt + ROS 2)  

The end result: A modular pipeline that translates human instructions like *“pick up the blue cube”* into executable **pick-and-place actions** performed by the robot.

---

## 🛠 Features
- Natural language → structured robot command conversion using GPT-4 Turbo.  
- RGB + depth camera input for **object grounding** in 3D space.  
- Back-projection of pixels into real-world coordinates with ROS2 TF transforms.  
- **Inverse kinematics & trajectory planning** via MoveIt.  
- Support for gripper actions (`open_gripper()`, `close_gripper()`), Cartesian moves, and lifting operations.  
- Modular ROS 2 node design for extensibility.  

---

## 📂 Project Structure
```text
vlm-robot-arm/
├── llm_command_listener.py    # Main controller: integrates GPT and PandaCommander
├── cordinate_converter.py     # Converts image pixel + depth → robot base coordinates
├── commands.py                # PandaCommander: motion planning + gripper control
├── gpt.py                     # GPT API wrapper with structured JSON prompt design
├── pose_moveit_controller.py  # Alternate node for direct Pose targets
├── move_group.launch.py       # ROS 2 launch file for MoveIt & RViz setup
└── Image_and_Video_Computing_Report.pdf  # Full project report
```

---

## ⚙️ System Architecture
1. **Input**:  
   - User provides a natural language command.  
   - Robot camera captures RGB + depth images of the environment.  

2. **Command Processing**:  
   - GPT-4 Turbo parses the instruction into a structured JSON plan:  
     ```json
     {
       "position": [u, v],
       "lift_height": 0.2,
       "actions": ["move_to()", "close_gripper()", "lift()"]
     }
     ```

3. **Perception & Projection**:  
   - Pixel coordinates `(u, v)` + depth are back-projected to 3D points.  
   - ROS2 TF transforms convert from **camera frame → robot base frame**.  

4. **Motion Planning**:  
   - MoveIt computes a feasible trajectory to the target (x, y, z).  
   - Constraints ensure **collision-free execution**.  

5. **Execution**:  
   - PandaCommander executes motions and gripper actions.  
   - Robot performs pick-and-place autonomously.  

<p align="center">
  <img src="docs/architecture.png" width="500"/>
</p>

---

## 🚀 Getting Started

### Prerequisites
- **ROS 2 Humble/Foxy** (tested with Humble)  
- **MoveIt 2**  
- **Gazebo / Isaac Sim** (optional for simulation)  
- **Python 3.10+**  
- OpenAI API key for GPT  

### Installation
```bash
# Clone this repository
git clone https://github.com/yourusername/vlm-robot-arm.git
cd vlm-robot-arm

# Install dependencies
pip install openai opencv-python numpy
sudo apt install ros-humble-moveit ros-humble-cv-bridge
