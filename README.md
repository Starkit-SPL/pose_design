# PoseDesign
The application for webots. It provides opportunities to save, load and design poses for Nao v6.

# How to install
cd ~/dev_ws

git clone https://github.com/Starkit-SPL/PoseDesign.git src/pose_design

colcon build --packages-select pose_design

. install/setup.bash

# How to run (another terminal)

1. start webots, run world nao_robocup.wbt
2. ros2 run nao_lola nao_lola

. install/setup.bash

ros2 run pose_design PoseDesign
