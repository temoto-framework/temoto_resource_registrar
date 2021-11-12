<<<<<<< HEAD
# RR Core implementation on ROS1

This is a implementation of the rr_core library on ROS1. It is meant to replace the existing resource registrar used inside TeMoto to allow for a more flexible and extendable resource management system.

This library was developed as part of a Masters thesis that can be found here:
[Design and Implementation of a Generalized Resource Management Architecture in the TeMoto Software Framework](https://www.ims.ut.ee/www-public2/at/2021/msc/atprog-courses-magistrit55-loti.05.036-allan-kustavus-text-20210520.pdf)

## How to install

This package contains of 3 parts:
* [The rr_core library](rr_core)
* The ROS1 extension


1. Move the repository to your workspace folder
3. Build the workspace
```
catkin build
```

## What is included

The ROS1 implementation includes a RR system example containing of [3 nodes](src):
* [A resource consumer]( src/ResourceConsumer.cpp), requireing resources.
* [A resource agent]( src/ResourceAgent.cpp), relaying consumer requests to producers.
* [A resource producer]( src/ResourceProducer.cpp), data generators for potential consumers.

These nodes to the following:
* The consumer requests a counter resource from the agent
* The agent relays this request to the producer
* The producer loads a resource and returns it. The resource is to send a status message every second.
* The agent relays the result to the consumer
* The consum recieves the message.
* Every second a status message is relayed as a chain of producer->agent->consumer.
* After 5 status messages the client requests a resource unload request to the agent.
* The agent resolves resource dependencies and sends a unload request to the producer.
* The producer unloads its resources.
* The agent unloads its resources.
* The client is notified of the unload.

To run the nodes following commands can be used for the producer, agend and consumer:
```
roscore;
rosrun temoto_resource_registrar producer;
rosrun temoto_resource_registrar agent;
rosrun temoto_resource_registrar consumer;
```
=======
# temoto_resource_registrar

This is a library designed to manage resources between different nodes inside a distributed system.

It has ROS1 and ROS2 implementation examples in the branches. Instructions can be found in their readme files:
* [ROS1](https://github.com/temoto-telerobotics/temoto_resource_registrar/tree/feature/ros1_implementation_prototype)
* [ROS2](https://github.com/temoto-telerobotics/temoto_resource_registrar/tree/feature/ros2_implementation_prototype)

This library was developed as part of a Masters thesis that can be found here:
[Design and Implementation of a Generalized Resource Management Architecture in the TeMoto Software Framework](https://www.ims.ut.ee/www-public2/at/2021/msc/atprog-courses-magistrit55-loti.05.036-allan-kustavus-text-20210520.pdf)
>>>>>>> main
