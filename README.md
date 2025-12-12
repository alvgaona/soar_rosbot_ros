# Soar ROSbot Maze

## Prerequisites

- Ubuntu 24.04 (noble)
- ROS 2 Jazzy
- Gazebo Harmonic

## Getting Started

The standard way of working with ROS 2 is to create your application workspace.
In this case, we can create a directory in `$HOME` but it could be anywhere.

```sh
mkdir -p ~/soar_ws/src
```
Then `cd` into `~/soar_ws/src` and clone the project.

To clone this repository and correctly initialize its submodules, run:

```bash
git clone --recurse-submodules https://github.com/alvgaona/soar-rosbot-maze.git
# If you forgot --recurse-submodules, or to update later:
# git submodule update --init --recursive
```

### Build

Once all submodules are cloned, you'll need to build the packages with `colcon`.

```sh
colcon build --event-hander console_direct+
```

The above command will look for all ROS 2 packages in the repository and build it. 
If succesful, you will see in `~/soar_ws/src` there folders: `build/`, `install/`, `log/`.

### Run the application

The very first thing is to set up Gazebo environment and spawn the ROSbot XL with its sensors.

```sh
ros2 launch soar_rosbot_gazebo soar_maze.launch.py
```

Afte that, you gotta start the perception stack which processes sensor information and transform it
to data that Soar will use to navigate the maze.

```sh
ros2 launch soar_rosbot_perception perception.launch.py
```

You can verify there are multiple topics from the above:

- `/aruco/detected`: true/false value whether the ArUco marker (goal) is detected by the camera.
- `/aruco/distance`: distance (in meters) of the ROSbot to the marker.
- `/wall/front`: true/false value whether there's a wall in front within a certain distance.
- `/wall/left`: true/false value whether there's a wall on the left within a certain distance.
- `/wall/right`: true/false value whether there's a wall on the right within a certain distance.

> [!IMPORTANT]
> The wall topics are with respect to the LiDAR frame. Meaning if the robot turns, the values published
> will change.

Lastly, just need to pull up the Soar controller with:

```sh
ros2 launch soar_rosbot_controller controller.launch.py
```

## Common Issues

WIP
