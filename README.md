# Sharework DEMO

Sharework demo includes the following modules:

- Robot control, perception, motion planning modules
- Action planning modules
- Task planning module
- HMI modules

## Robot control, perception, motion planning modules

For simulation:

`roslaunch sharework_cembre_configurations fake_start.launch`

For real demo:

- Turn on the robot
- Terminal 1 (launch robot controllers, RVIZ, and planning pipeline):
  - `sudo su`
  - `roslaunch sharework_cembre_configurations real_start.launch`


- Terminal 2 (launch vision system and skeleton tracking):
  - `ssh .....`
  - `roslaunch vision`


## Action planning modules

Launch the manipulation framework and spawn the movable objects.

Terminal 3:

`roslaunch sharework_cembre_skills skills.launch`

## Task planning module

Launch mongo daemon:

`mongod --dbpath ~/data/db`

Launch task planning interface:

`roslaunch sharework_task_planning hrc_fake.launch`

Launch platinum or dispatcher

## HMI modules

- Open Sharework app on tablet and login.
- Connect to ROSbridge (IP: 192.168.10.33).

To load the custom figures for the tasks you should do the following:
- Create a .png file (Aspect ratio 16:9, recommended resolution 1600x900) named after the Task ID you want (H_01.png, R_02.png, etc).
- Go to LocalStorage/Android/data/com.lms.ShareworkModule9/files.
- Save or copy the png created inside the folder.
