# part_perception package in ARIAC
The video of last qualifier, 3b has been uploaded to youku. 
Please click here: [youku](https://v.youku.com/v_show/id_XMzczODQ5MDA2OA==.html?spm=a2h3j.8428770.3416059.1)

## 0. Brief:
This repository contains the package implemented by Shaotu Jia. The whole project contains three packages, other packages are implemented by Yuxin Ma and Zejiang Zeng.  

## 1. Overview of Team Approach: 
Our team implemented three nodes to accomplish the ARAIC competition, order_manager, part_perception, and move_srv. The below Figure shows the how each node communicates with other in ROS system. In ROS system, nodes can communicate via rostopic and rosservice. Order_manager is the high-level order that plans which kit should be picked up. Part_perception node uses a logical camera to detect the part on the conveyor belt and the offset between part and gripper when a part is attaching to the gripper. Move_srv node controls the action of UR10; it provides several modes to finish different actions, such as picking a part from bins, picking a part from the conveyor belt, and place gripped part on the desired location. 
![Image text](https://github.com/ShaotuJia/part_perception/blob/master/images/ARIAC%20Architecture.png)

In this ARIAC competition, we use only one logical camera on the conveyor belt (Figure below). This logical camera is used to detect the parts on the belt and the offset between part and gripper. The order_manager will look up the order and decides which part should be picked up first based on the kit priority.  Once the order_manager knows which part should be picked up, it will prefer to pick this part from the belt; otherwise, it will pick this part from bins. When picking parts from bins, we make some assumptions for the part layout. For example, we assume gear layout as 4x4, 3x3, or 2x2. We try the first layout at first. If the UR10 get anything in the 4x4 layout, it will try another layout until it grips a part. After picking the part, the gripper will go to the logical camera to check the offset between part and gripper.  
![Image text](https://github.com/ShaotuJia/part_perception/blob/master/images/Sensor_Configuration_in_Gazebo_Environment.jpg)

## 2. Individual Contribution:
The part_perception package uses logical cameras to detect parts on the belt, gripper, and AVG. It contains three classes belt_inventory, part_offset_gripper, and incorrect_part_agv, to detect parts on the belt, gripper, and AVG, respectively. The class incorrect_part_agv is only used in testing. In the final version of our code, we disable the incorrect_part_agv and remove the logical camera on the top of AGV. Once logical cameras obtain useful information, they will send this information to order_manager and move_srv. 

### 2.1 Belt_Inventory:
The class belt_inventory have two functions. First, it publishes a rostopic /ariac/belt_inventory. This rostopic contains all parts that currently on the conveyor belt and also within the workspace of UR10. Second, it provides a rosservice /ariac/query_part to predicate the part location in the future time. Users give a request with part type and future time. Then, the rosservice /araic/query_part responds the location of this part in that future moment if the part is still within the workspace of UR10; otherwise, the rosservice will respond false and let you know there is no that part in that future moment which can be picked up by UR10.  

#### 2.1.1 rostopic /ariac/ belt_inventory:
The class Belt_Inventory first subscribes rostopic /tf. The rostopic /tf contains all relative tf transform at the current time. The beauty of rostopic /tf can assign different frame_id for the same type of part. For example, we have two gears that have detected by logical camera 1. In rostopic /tf, the first detected gear will be logical_camera_1_gear_1_frame; and the second gear will be logical_camera_1_gear_2_frame. Thus, we can easily recognize the difference between two same type parts. However, the rostopics published by the logical camera cannot assign different frame_id for same type parts.

When subscribing rostopic /tf, we need a filter to remove useless information and obtain the part information relative to the logical camera(line 127 ~ 131 part_perception.cpp). Once we find a part that currently under the logical camera, we will check whether this part is already in the belt_inventory list. If this part is not in the belt_inventory list, we add it to the list. If this part already exists in the belt_inventory, the obtain the part information in the belt_inventory, first. And then, compute the timestamp difference between current part info newly detected and previous part info stored in the belt_inventory; and also compute the location difference same as timestamp difference. Next, we can use the location difference and time difference to compute the velocity of the conveyor belt. Finally, we will repeat the previous steps to add new parts in the belt_inventory and update the parts stored in the belt_inventory using belt velocity and the current time; if any part is out of the workspace of UR10, we need to remove it from the belt_inventory list.

#### 2.1.2 rosservice /araic/query_part:
This rosservice predicates the part location in the future time. When users send a request with part type, and future time, this service will search belt_inventory list to find the same part type. If the belt_inventory list has the same part as the request, we will use the conveyor belt velocity to compute the part location in that future time. If the part location in that future time is within the workspace of UR10, the rosservice will return the part location in that future time. If there is no same part in belt_inventory or the part location in that future time is out of the UR10 workspace, the rosservice will return false with an error message.

### 2.2 Part_offset_gripper:
This class checks the offset between part and gripper when a part is attached to the gripper. When UR10 is picking parts from the conveyor belt and bins, it is very difficult to grip a part very accurate; there is an offset between part and gripper. If we ignore this offset, the final position on the AGV will be inaccurate, too. Thus, we need to know the offset and adjust the final position of gripper based on this offset.

#### 2.2.1 rostopic /ariac/part_offset_gripper:
Same to class Belt_Inventory, this class also subscribes rostopic /tf for the same reason. When UR10 places the attached part under the logical camera 1, /tf will contain the transformation between attached part and gripper. In this case, we need a filter the obtain this transformation from /tf and this transformation is the offset between attached part and gripper. Finally, we will publish this transformation to rostopic /ariac/part_offset_gripper. When the gripper with attached part leaves the range of logical camera 1, this rostopic will stop publishing messages.

#### 2.2.2 rosservice /ariac/check_part_offset:
When we have the transformation from 2.2.1, we also create a rosservice to send out this information. Users can call rosservice /ariac/check_part_offset; the server will response the offset if the gripper is under the logical camera 1; if the gripper leaves the range of the logical camera, the server will response false with an error message.

### 2.3 Incorrect_part_agv: 
This class is only used for testing, and we disable this class in the final configuration. This class checks whether the part positions in AGV are correct. A logical camera 2 is placed on the top of AGV. We will compare the part position to the position in the order.

#### 2.3.1 rostopic /ariac/incorrect_parts_agv_1:
When UR10 places parts on AGV, the logical camera 2 will publish the transformation between the part and logical camera 2. We also subscribe rostopic /tf. In this case, the /tf will contain the transformation between part on AGV and logical camera 2. We extract these transformations same as the previous class. Next, we will look up the transformation between parts and AGV load point by tf tree. Finally, we will compare the transformations between parts and AGV load point to the relative position in /ariac/order. If any part in an incorrect position, the incorrect position will be published via rostopic /araic/incorrect_parts_agv_1.

#### 2.3.2 rosservice /ariac/check_part_pos_agv_1:
There is also a server for the users who prefer to use rosservice. This server subscribes the rostopic /ariac/incorrect_parts_agv_1. When users call this rosservice, it will response the part positions which are incorrect. If all parts are in the correct position, this server will only response a message.    

