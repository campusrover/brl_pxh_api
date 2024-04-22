# BRL PX-100 API

This is the Brandeis Robotics Lab's API for the Interbotix PX-100
robotic arm.

---
## Table of Contents
- [Installation](#installation)
- [What](#what)
- [Why](#why)
- [How](#how)
---

## Installation

### Create a New Workspace

We recommend starting with a fresh catkin workspace.

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

### Git Clone the API repository


### Testing


## What

## Why

## How
