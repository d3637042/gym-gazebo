# An OpenAI gym extension for using Gazebo known as `gym-gazebo` added 

## Installation
Refer to [INSTALL.md](INSTALL.md)

## Usage

Refer to [README.md](gym-gazebo/docker/README.md)

### Display reward plot

Display a graph showing the current reward history by running the following script:

```bash
cd examples/utilities
python display_plot.py
```

HINT: use `--help` flag for more options.

### Killing background processes

Sometimes, after ending or killing the simulation `gzserver` and `rosmaster` stay on the background, make sure you end them before starting new tests.

We recommend creating an alias to kill those processes.

```bash
echo "alias killgazebogym='killall -9 rosout roslaunch rosmaster gzserver nodelet robot_state_publisher gzclient'" >> ~/.bashrc
```
