World 	{  X:<[0, 0, 0, 1, 0, 0, 0]> }                         

table_base (World) {
    Q:[0 0 .6]
    shape:marker, size:[.03],
}
table (table_base){
    shape:ssBox, Q:[0 0 -.05], size:[2.3 1.24 .05 .02], color:[.3 .3 .3]
    contact:1, logical:{ }
}

_obs (World) 	{  type:ssBox, size:[3 0.2 1 .005], contact:1 Q:<[  0.0, -0.7, 1.1, 1, 0, .0, .0]> color:[0, 0, 0, 0.1]}

Prefix: "a0_"
#Include: 'franka.g'
Include: 'franka_vacuum_restricted.g'

Edit a0_base (table) {Q:<t(-.4 -.3 .05) d(90 0 0 1)>}

goal1 (table_base){ type:ssBox, size:[0.2 0.2 0.1 .002], contact:0 Q:<[  -0.4, 0.1, 0.03, 1, 0, .0, 0]> color:[0.4, 1, 1, 0.3]}
goal2 (table_base){ type:ssBox, size:[0.15 0.2 0.1 .002], contact:0 Q:<[  -0.4, 0.1, 0.14, 1, 0, .0, 0]> color:[0.4, 1, 0.4, 0.3]}
goal3 (table_base){ type:ssBox, size:[0.1 0.15 0.1 .002], contact:0 Q:<[  -0.4, 0.1, 0.25, 1, 0, .0, 0]> color:[1, 1, 0.4, 0.3]}
goal4 (table_base){ type:ssBox, size:[0.1 0.1 0.1 .002], contact:0 Q:<[  -0.4, 0.1, 0.36, 1, 0, .0, 0]> color:[1, 0.1, 0.1, 0.3]}
#m (goal){ type:marker, size:[0.2], Q:<[  0, -0, 0., 1, 0, .0, 0]> color:[0.9, 0.1, 0.1, 1]}

#m (a0_gripper_vis){ type:marker, size:[0.2], Q:<[  0, -0, 0., 1, 0, .0, 0]> color:[0.9, 0.1, 0.1, 1]}

#m (obj1){ type:marker, size:[0.2], Q:<[  0, -0, 0., 1, 0, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
obj1 (table_base){ joint:rigid type:ssBox, size:[0.2 0.2 0.1 0.01], contact:1 Q:<[  -0.8, .175, 0.03, 1, 0, .0, 0]> color:[0.4, 1, 1, 1]}
obj2 (table_base){ joint:rigid type:ssBox, size:[0.15 0.2 0.1 .01], contact:1 Q:<[  -0.2, .375, 0.03, 1, 0, .0, 0]> color:[0.4, 1, 0.4, 1]}
obj3 (table_base){ joint:rigid type:ssBox, size:[0.15 0.15 0.1 .01], contact:1 Q:<[  -0.1, .1, 0.03, 1, 0, .0, 0]> color:[1, 1, 0.4, 1]}
obj4 (table_base){ joint:rigid type:ssBox, size:[0.1 0.1 0.1 .01], contact:1 Q:<[  -0.6, 0, 0.03, 1, 0, .0, 0]> color:[1, 0.1, 0.1, 1]}

#_wall1 (table_base){ type:ssBox, size:[0.05 0.45 0.2 .01], contact:1 Q:<[  -0.2, .1, 0.05, 1, 0, .0, 0]> color:[0.9, 0.9, 0.9, 1]}
#_wall2 (table_base){ type:ssBox, size:[0.5 0.05 0.2 .01], contact:1 Q:<[  -0.5, .6, 0.05, 1, 0, .0, 0]> color:[0.9, 0.9, 0.9, 1]}
#_wall3 (table_base){ type:ssBox, size:[0.05 0.45 0.2 .01], contact:1 Q:<[  -1, .1, 0.05, 1, 0, .0, 0]> color:[0.9, 0.9, 0.9, 1]}
