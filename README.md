# Autoware Universe - CARLA Simulator
---------------------------------------------------------------------------------------------------------------------------------------------

## h2_i/galactic
ENV:
- Ubuntu 20.04
- ROS2: Galactic
- Nvidia Driver Version: 565.57.01
- Cuda compiler driver: release 11.6, V11.6.55

- Autoware Universe: Galactic
- CARLA Simulator: 0.9.13
  -> If you want to modify or create CARLA maps, you need Source Code CARLA \ That File Download https://github.com/carla-simulator/carla/releases/tag/0.9.13 or SSD 
  
- Unreal_Engine: 4.26

- op_bridge: ros2
- op_agent: ros2
- scenario_runner: openplanner_carla_bridge 

---------------------------------------------------------------------------------------------------------------------------------------------

## h2_i/humble
ENV:
- Ubuntu 22.04
- ROS2: Humble
- Nvidia Driver Version: 570.86.16
- Cuda compiler driver: release 11.6, V11.6.55

- Autoware Universe: tag/2023.10
- CARLA Simulator: 0.9.15
  -> If you want to modify or create CARLA maps, you need Source Code CARLA \ That File Download https://github.com/carla-simulator/carla/releases/tag/0.9.13 or SSD 
  
- Unreal_Engine: 4.26

- op_bridge: ros2-humble
- op_agent: ros2-humble --> **need autoware_contents** (in notion.so) 
- scenario_runner: openplanner_carla_bridge

---------------------------------------------------------------------------------------------------------------------------------------------

## h2_i/humble(aw_humble)
ENV:
- Ubuntu 22.04
- ROS2: Humble
- Nvidia Driver Version: 570.86.16
- Cuda compiler driver: release 11.6, V11.6.55

- Autoware Universe: humble
- CARLA Simulator: 0.9.15
  -> If you want to modify or create CARLA maps, you need Source Code CARLA \ That File Download https://github.com/carla-simulator/carla/releases/tag/0.9.13 or SSD 
  
- Unreal_Engine: 4.26

- op_bridge: ros2-humble
- op_agent: ros2-humble
- scenario_runner: openplanner_carla_bridge

