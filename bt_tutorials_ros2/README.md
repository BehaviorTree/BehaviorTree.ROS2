# bt_tutorials_ros2

## Overview

This package provides fully documented, runnable examples of the ROS 2 integration tutorials from the official [BehaviorTree.CPP documentation](https://www.behaviortree.dev/), allowing users to test, modify, and extend these examples in their own projects. Specifically, the following two nodes are created:

- **Fibonacci Action Client**: A BehaviorTree client node interacting with the `\fibonacci` action server from the official ROS2 examples.
- **AddTwoInts Service Client**: A BehaviorTree client node interacting with the `\add_two_ints` service server from the official ROS2 examples.

## Requirements

- **C++17**: Required for building the package.
- **ROS 2**: Tested on ROS 2 Humble.
- **BehaviorTree.CPP**: Version 4.0 or higher.
- **BehaviorTree.ROS**, i.e. this repo.

## Compiling
Compile using `colcon`, for instance as:
```
colcon build --packages-select bt_tutorials_ros2
```
## Running

### Fibonacci Action Client

Ensure the `/fibonacci` action server is running. This server is typically included with the ROS 2 installation and can be started using the following command:

```bash
ros2 run action_tutorials_cpp fibonacci_action_server
```
If the server is not available, you can follow the official ROS 2 tutorial [here](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html#writing-an-action-server) to compile and run it.

Once the action server is up, you can run the **Fibonacci BehaviorTree action client** node with:
```
ros2 run bt_tutorials_ros2 bt_tutorials_ros2_action_client_node
```

### AddTwoInts Service Client
Ensure the `/add_two_ints` service server is running. You can implement, compile, and run the server following the official ROS 2 tutorial [here](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html). For example, you can run the server with:
```
ros2 run cpp_srvcli server
```

Once the service server is up, you can run the **AddTwoInts BehaviorTree service client** node with:
```
ros2 run bt_tutorials_ros2 bt_tutorials_ros2_service_client_node
```

### Author
Janak Panthi (Crasun Jans)
