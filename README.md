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

# Setting up ar_tags
1. To start a camera, simply call roslaunch launch file that corresponds to the camera you want to turn on (Ex: head_camera.launch). 
2. To verify this is working, you can call `roslaunch image_view image_view image:=/cameras/head_camera/image` (Or /cameras/left_hand_camera/image)
3. Since head cameras are not calibrated correctly, you must run ./headsub.py which is in the src folder to re-broadcast all ar_tag values that's read from the head camera, by calling ./headsub.py. This reads the conf.py file to determine which ar tags are being used and rebroadcasts them.
4. You will also need to download the ar_track_avalar package, which you can follow lab instructions for.

# Changing ingredients/ Menu
Modify the ingredient_list or menu_list in conf.py

# Boba Bot Usage
LAUNCH FILES WOO

1. Source devel/setup.sh
2. roslaunch boba_bot boba_bot.launch
3. A new terminal will automatically open, waiting for Baxter commands.
