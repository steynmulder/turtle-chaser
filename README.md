# Turtle Chaser
In this project two turtlebots in the ROS2 turtlesim environment will be chasing after each other. Specifically, there will be one `turtle_runner` who will run away from the `turtle_chaser`. 

## Implementation
The project is implemented in ROS2 Humble using C++. It consists of two Docker containers. 

One of these has two custom ROS2 nodes from the `turtle_command` package. One of these is the `turtle_runner`, which is a publisher to the `turtle1/cmd_vel` topic, providing random angular velocities in a `Twist` message. The other node is the `turtle_chaser`, which subscribes to the `Pose` topics of both turtles, publishes a `Twist` to `turtle2/cmd_vel` to chase after the runner, and handles three service clients, `spawn`, `turtle2/set_pen`, and `turtle1/teleport_absolute`. The first will create the additional turtle, the chaser, upon initialization of the simulation. The second will set the color of the trail of the chaser turtle, and the third will set the position of the runner turtle to a random new location whenever the runner is 'tagged' by the chaser.

The other Docker container runs the `turtlesim_node` from the `turtlesim` package, commonly used for ROS2 tutorials. This environment was chosen, because the aim of the project is to show the communication between ROS2 nodes in different Docker containers.

![Diagram for Docker Containers and ROS2 Nodes](./images/diagram.svg)

## Usage

After cloning and entering the repo, permission needs to be granted to Docker to access the display, as to use the GUI provided by `turtlesim`.

```bash
xhost +local:docker
```

Next, simply use `docker-compose` to build both containers.
```bash
docker-compose up --build
```

The simulation should now build and run. The turtle in the center will start moving in random directions, while the one in the bottom left, with the red trail, will start moving after it. After it has reached the runner, the runner will be teleported to a new position and the chaser will continue chasing.

If the containers need to be run again, it might be needed to run
```bash
docker container prune
```
before running `docker-compose` again.