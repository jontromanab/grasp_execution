# grasp_execution
====
Action server which will accept grasp_execution/grasp by an action client and execute the grasp

***Only applied for UR5 arm and robotiq gripper. Can change to any other robot easily. Please refer to grasp_ws package for URDF and details about robot setup***

*** now added functions specifically for PR2 robots. Please look at relatively new commits ***

The grasp_execution_client is just a test file. In all the grasping algorithm packages, there should be a client which can call the grasp_execution_server to execute a single grasp in turn
