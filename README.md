# part_perception package
## build belt_inventory
- rostopic: /ariac/belt_inventory
- rosservice: /ariac/query_part
## find incorrect part position on AVG 1
- rostopic: /ariac/incorrect_parts_agv_1
- rosservice: /ariac/check_part_pos_agv_1
## check part offset on gripper
- rostopic: /ariac/part_offset_gripper
- rosservice: /ariac/check_part_offset
