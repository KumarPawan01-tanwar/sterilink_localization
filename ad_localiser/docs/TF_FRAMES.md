# TF FRAMES

## CONSIDERATIONS

- A parent frame can have multiple child frame, but each child can only have one parent.  

<img src="./TF_consideration1.png" width="600">

- An odometry topic can be sent to indirectly linked parent and child frame.  

<img src="./TF_consideration2.png" width="600">


## RESULTING FRAMES
### ODOM SELECTION = 1 (ODOM_OPTITRACK_ONLY)
<img src="./TF_odomselection1.png">

### ODOM SELECTION = 2 (ODOM_ICP_ONLY)
<img src="./TF_odomselection2.png">  

### ODOM SELECTION = 3 (BOTH)
<img src="./TF_odomselection3.png">  