# BRL PX-100 API

This is the Brandeis Robotics Lab's API for the Interbotix PX-100
robotic arm.

---
## Table of Contents
- [Installation](#installation)
    - [Create a New Workspace](#create-a-new-workspace)
    - [Git Clone and Build the BRL API ROS Package](
#git-clone-and-build-the-brl-api-ros-package)
    - [Create the Client ROS Package](#create-the-client-ros-package)
    - [Testing Your Setup](#testing-your-setup)
    - [Next Steps](#next-steps)
- [Simulation Mode](#simulation-mode)
- [A note on an error](#a-note-on-an-error)
---

## Installation

### Create a New Workspace

We recommend starting with a fresh catkin workspace. In a new terminal
window, execute:

```bash
mkdir -p ~/your_new_ws/src
cd ~/your_new_ws
catkin_make
```

Next, while still in `~/your_new_ws`,

```bash
source devel/setup.bash
```

This last command has to be executed each time you launch a new
terminal. To automate this, add:

```bash
source ~/your_new_ws/devel/setup.bash
```

as the _last line_ in your `.bashrc` file. As the emphases suggest,
it's important that this command is literally the _very last line_ of
your `.bashrc` file. 
 
It's very likely that this isn't the first and only catkin workspace on
your computer. That's okay! Say that you have another workspace on your
computer called `your_old_ws` that you want to switch to.

Then, simply modify the very end of your `.bashrc` file as below:

```bash
# Source your_new_ws
# source ~/your_new_ws/devel/setup.bash
# Source your_old_ws
source ~/your_old_ws/devel/setup.bash
```

Whenever you want to switch to `your_new_ws` again, simply do:

```bash
# Source your_new_ws
source ~/your_new_ws/devel/setup.bash
# Source your_old_ws
# source ~/your_old_ws/devel/setup.bash
```

This pattern can be used for as many workspaces as you have on your
computer!

### Git Clone and Build the BRL API ROS package

With `your_new_ws` set up and properly sourced, navigate to its `src`
directory with:

```bash
cd ~/your_new_ws/src
```

and git clone the API ROS package:

```bash
git clone git@github.com:campusrover/brl_pxh_api.git
```

Now move to `~/your_new_ws` and build the package:

```bash
cd ~/your_new_ws
catkin_make
```

At this point, the BRL API should have been successfully installed in
your workspace!

### Create the Client ROS Package 

Next, create the ROS package where you will write the code that
will use the BRL PX-100 API. You can name the package whatever you
want, but it's important that you add the `brl_pxh_api` package as a
dependency.

For instance, if your package's name is `your_pkg`, you can create
the package with the commands below:

```bashrc
cd ~/your_new_ws
catkin_create_pkg your_pkg std_msgs rospy roscpp brl_pxh_api
```

and build the package with:

```bashrc
cd ~/your_new_ws
catkin_make
```

### Testing Your Setup

___TO-DO: Update this section after successfully connecting the PX-100
to the Pi4B___

Now we'll test if the API and your client package have been set up
correctly. 

First, copy the `brl_api_tester.py` file in the `/src/brl_pxh_api/`
directory of the `brl_pxh_api` package into the `src` directory of
`your_pkg`. You can do this with the bash command:

```bash
cp ~/your_new_ws/src/brl_pxh_api/src/brl_pxh_api/brl_api_tester.py \
~/your_new_ws/src/your_pkg/src/
```

Second, execute:

```bash
roslaunch brl_pxh_api brl_pxh_api.launch
```

This should launch an RViz window, and prime the arm so that it's ready
to receive commands.

Third, in a new terminal, execute:

```bash
rosrun your_pkg brl_api_tester.py
```

You should now see the arm executing a sequence of movements in a loop.
When you're satisfied with what you see, press Control-C in the
terminal where you executed the `rosrun` command.

### Next Steps

The `brl_api_tester.py` file is your friend. It shows you the basic
steps of using the BRL PX-100 API. First, you need to import the
BrlPxhClient:

```python3
from brl_pxh_api.brl_pxh_client import BrlPxhClient
```

Then you should create a `BrlPxhClient` object, and assign it to a
variable that you'll use whenever you want to invoke the API. In the
`brl_api_tester.py` file, we make it an attribute of the
`BrlPxhAPiTester` class:

```python3
self.api_client = BrlPxhClient()
```

And now you're ready to use the arm! For instance, if you want to open
then close the arm's gripper, you can execute:

```python3
self.api_client.brl_open_gripper()
self.api_client.brl_close_gripper()
```

For an example that uses the Brl API more extensively, see the
[PNP](https://github.com/campusrover/pnp) project.

## Simulation Mode

To use the BRL PX-100 API in simulation mode, first use your favorite
text editor to open the `brl_pxh_api.launch` file. You can find the
file under the `launch` directory of the `brl_pxh_api` package. Next,
navigate to the line that reads:

```
<arg name="use_sim" value="false" />
```

and change it to:

```
<arg name="use_sim" value="true" />
```

To test that the API will operate in simulation mode as intended,
execute:

```bash
roslaunch brl_pxh_api brl_pxh_api.launch
```

and then, in another terminal window, enter: 

```bash
rosrun brl_pxh_api brl_api_tester
```

You should now see the simulated model of the arm moving as directed by
the `brl_api_tester` in RViz.

