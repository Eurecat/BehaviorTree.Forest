# BehaviorTree Forest

This repository takes leverage of the following repositories
- [behaviortree_cpp (v 4.6.2)](https://github.com/BehaviorTree/BehaviorTree.CPP/tree/4.6.2)
- [behaviortree_eut_plugins](https://github.com/Eurecat/behaviortree_eut_plugins.git)
- [behaviortree_ros2](https://github.com/Eurecat/behaviortree_ros2.git)

In particular, it provides the infrastructure to implement:
- a server capable of maintaining some sync entries in a centralized storage
- capability of spawning/despawning behavior trees on demand, each one running in a separate process and ros2 node
- through sync entries, trees can essentially exchange data seamlessly

The approach is thought to boost encapsulation and reusability, while going towards a correct separation of concerns by avoiding repetition of the same trees within the each one or mixing different capabilties orchestration logics and states in the same behavior tree that tries to govern it all which is not scalable.

## How to use it
You should start by running the main BehaviorTreeForest Server
```
ros2 run behaviortree_forest behaviortree_server
```
Optionally, you can already start it and reference a yaml file for initial

# Documentation

Call Eurecat Robotics