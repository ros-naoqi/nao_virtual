nao_v4_moveit_config
====================

This moveit package is based on a textured nao :
https://github.com/vrabaud/nao_meshes_generator
The urdf used for the generation of the configuration file is embedded in this package. It's the same that the one in nao_description : https://github.com/keulYSMB/nao_robot/tree/devel/nao_description/urdf/naov4_generated_urdf


You can run this moveit package either unconnected to any robot or attached to a robot -real or simulated):
For a standalone execution :
roslaunch nao_v4_moveit_config demo.launch
To launch it on a real nao : 
roslaunch nao_dcm_bringup nao_dcm_H25_bringup_remote.launch << launch nao_bringup
roslaunch nao_v4_moveit_config moveit_planner.launch

To launch it on a gazebo simulated nao (in gazebo):
roslaunch nao_dcm_gazebo nao_dcm_gazebo_H25.launch
roslaunch nao_v4_moveit_config moveit_planner.launch

This is based on the work of Konstantinos Chatzilygeroudis.

TODO : create our own nao_gazebo and nao_bringup to use moveit with ALMotion and not only DCM

