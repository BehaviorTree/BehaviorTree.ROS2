# BehaviorTree.ROS2
[![Test](https://github.com/BehaviorTree/BehaviorTree.ROS2/actions/workflows/test.yml/badge.svg)](https://github.com/BehaviorTree/BehaviorTree.ROS2/actions/workflows/test.yml)

This repository contains useful wrappers to use ROS2 and BehaviorTree.CPP together.

In particular, it provides a standard way to implement:

- Action clients.
- Service Clients.
- Topic Subscribers.
- Topic Publishers.

Our main goals are:

- to minimize the amount of boilerplate.
- to make asynchronous Actions non-blocking.

Note that this library is compatible **only** with:

- **BT.CPP** 4.1 or newer.
- **ROS2** Humble or newer.

Additionally, check **plugins.hpp** to see how to learn how to
wrap your Nodes into plugins that can be loaded at run-time.


## Acknowledgements

A lot of code is either inspired or copied from [Nav2](https://navigation.ros.org/).

For this reason, we retain the same license and copyright.


