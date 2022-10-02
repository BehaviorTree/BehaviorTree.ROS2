# BehaviorTree.ROS2


## After starting up tycrawler run the following

```
cd ~/ros_ws/src
git clone https://github.com/TechnoYantra/BehaviorTree.ROS2.git
git checkout nav2_action_node
cd ~/ros_ws
colcon build
. install/setup.bash
ros2 run behaviortree_ros2 nav2_client 

```

It executable not found then 
```
./build/behaviortree_ros2/nav2_client
```
