# PoseDesign
The application for webots. It provides opportunities to save, load and design poses for Nao v6.

# How to install
cd ~/dev_ws

git clone https://github.com/Starkit-SPL/pose_design.git src/pose_design

colcon build --packages-select pose_design

. install/setup.bash

# How to run (another terminal)

1. start webots, run world nao_robocup.wbt
2. ros2 run nao_lola nao_lola

. install/setup.bash

ros2 run pose_design pose_design

# If you want to change packet code (on VS code)

1. On VS code "Get Started" window tap clone git repository.
2. Place the URL in corresponding line on the top:

https://github.com/Starkit-SPL/pose_design.git

3. Select repository folder as path_to_your_ros2_ws/src
4. Tap open in right bottom corner, then tap trust.
5. So, if you want to load latest version, ->VCS menu->...->fetch.
