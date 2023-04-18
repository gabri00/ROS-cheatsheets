# ROS 2 (Humble)

## ROS 2 commands

## Building packages and preparing workspace

Build workspace/package:
```sh
colcon build --symlink-install # Saves you from having to rebuild every time you tweak the code

colcon build --packages-up-to <pkg> # Builds a package, plus all its dependencies

colcon build --packages-select <pkg> # Builds only the selected package
```

Resolve dependencies:
```sh
rosdep install -i --from-path src --rosdistro humble -y
```

Create package:
```sh
ros2 pkg create --build-type ament_python  <pkg_name>
```

List executables in a package:
```sh
ros2 pkg executables <pkg_name>
```

## Nodes

Run a node:
```sh
ros2 run <pkg_name> <node_name>
```

List active nodes:
```sh
ros2 node list
```

Show more info on a node (subscribers, publishers, services, and actions):
```sh
ros2 node info <node_name>
```

## Topics

List active topics:
```sh
ros2 topic list
ros2 topic list -t # Displays also the type
```

Show the data being published on a topic:
```sh
ros2 topic echo <topic_name>
```

Shows the data structure of of the message:
```sh
ros2 interface show <msg_type>
```

Publish data onto a topic directly from the command line:
```sh
ros2 topic pub --once <topic_name> <msg_type> "<args>"
topic pub -- rate 10 <topic_name> <msg_type> "<args>"
```

View the rate at which data is published:
```sh
ros2 topic hz <topic_name>
```

## Services

## Actions

## Launch files

Run a launch file:
```sh
ros2 launch <pkg_name> <launch_file_name>
```