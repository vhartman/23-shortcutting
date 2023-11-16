Include: '../../rai-robotModels/panda/panda_restricted.g'

## zero position

Edit panda_joint2 { q= -.5 }
Edit panda_joint4 { q= -2 }
# delete original gripper   
 
Delete panda_link8>panda_hand_joint
Delete panda_hand_joint
Delete panda_hand_1
Delete panda_hand_0
Delete panda_hand>panda_finger_joint1
Delete panda_hand>panda_finger_joint2
Delete panda_finger_joint1
Delete panda_finger_joint2
Delete panda_leftfinger_1   
Delete panda_leftfinger_0
Delete panda_rightfinger_1  
Delete panda_rightfinger_0
Delete panda_coll_hand
Delete panda_coll_finger1
Delete panda_coll_finger2   

Delete gripper
Delete palm
Delete finger1
Delete finger2

#gripper (panda_joint7){
#    shape:sphere, size:[.03]
#    Q:<t(.11 0.0 .0)>
#    contact:0 
#    color:[1.0,0.5,0.5,1.0]
#}

# add robotiq
#Include: '../../../rai-robotModels/robotiq/robotiq.g'
#Edit robotiq_base (panda_joint8) { Q:[0 0 .05] }

# filling the hole
gripper_fill (panda_joint8){ shape:cylinder, color:[.1, .1, .1 ,1], size:[.17 .031]}

gripper (panda_joint7){
    shape:sphere, size:[.02]
    Q:<t(.2 0.0 .0)>
    contact:0 
    color:[1.0,0.5,0.5,1.0]
}

#gripper (panda_joint7){
#    shape:sphere, size:[.02]
#    Q:<t(.125 0.0 .0)>
#    contact:0 
#    color:[1.0,0.5,0.5,1.0]
#}

gripper_p1 (panda_joint7){
    shape:sphere, size:[.02]
    Q:<t(0.2 -0.03 0.01)>
    contact:0
    color:[1.0,0.5,0.5,1.0]
}

gripper_p2 (panda_joint7){
    shape:sphere, size:[.02]
    Q:<t(0.2 0.03 0.01)>
    contact:0
    color:[1.0,0.5,0.5,1.0]
}

gripper_p3 (panda_joint7){
    shape:sphere, size:[.02]
    Q:<t(0.2 -0.0 -0.04)>
    contact:0
    color:[1.0,0.5,0.5,1.0]
}

gripper_vis (panda_joint7){
    shape:cylinder, size:[.04 0.05]
    Q:<t(0.2 0.0 0.00) d(90 0 1 0)>
    contact:0
    color:[1.0,0.5,0.5,.5]
}

gripper_coll (panda_joint7){
    shape:cylinder, size:[.03 0.05]
    Q:<t(0.2 0.0 0.0) d(90 0 1 0)>
    contact:0
    color:[1.0,0.5,0.5,.5]
}
