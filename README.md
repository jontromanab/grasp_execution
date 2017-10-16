# grasp_execution
====
Action server which will accept grasp_execution/grasp by an action client and execute the grasp

***Aapplied for UR5 arm and robotiq gripper. Can change to any other robot easily. Please refer to grasp_ws package for URDF and details about robot setup***
![1_orig](https://user-images.githubusercontent.com/3790876/31592011-0a714fa6-b1da-11e7-9458-2b90c6e1399a.png)

***Now added functions specifically for PR2 robots. Please look at relatively new commits***


![untitled](https://user-images.githubusercontent.com/3790876/31592062-615fabc8-b1da-11e7-9d1f-c8d7e4876cf4.png)

The grasp_execution_client is just a test file. In all the grasping algorithm packages, there should be a client which can call the grasp_execution_server to execute a single grasp in turn
