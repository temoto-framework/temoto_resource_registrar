# RR Core implementation POC on ROS2

This is a example implementation of the rr_core library on ROS2. Functionalities
supported by ROS1 were ported over to ROS2 as an example for the libraries
flexibility.

## How to install

This package contains of 3 parts:
* [The rr_core library](rr_core)
* The ROS2 extension
* [The ROS 2 communication interfaces](rr_interfaces)


1. Firstly rr_core needs to be built and installed according to its instructions
2. The ROS2 extension and the communication interfaces need to be placed inside a ROS2 workspace
3. Run to build the workspace.
```
colcon build
```

## What is included

This POC includes a RR system containing of [3 nodes](temoto_resource_registrar/src):
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
ros2 run temoto_rr producer;
ros2 run temoto_rr agent;
ros2 run temoto_rr consumer;
```
