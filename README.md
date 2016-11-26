# EE106a_Final_Project
ROS code to control a boba drink mixing Baxter Robot

# ROS Setup Instructions
# READ BEFORE CLONING THIS REPO
I assume you already have ROS setup


1. Create a folder for your ROS workspace. Make sure it includes a `src` subfolder. 
  * Ex: `mkdir -p ~/ee106a_final/src`
2. `cd ~/ee106a_final/src`
3. `catkin_init_workspace`
4. Clone this repo. `git clone https://github.com/yizow/boba_bot.git`
  * Feel free to use ssh to clone if you have ssh-keys setup.
5. `cd ~/ee106a_final/`
6. `catkin_make`