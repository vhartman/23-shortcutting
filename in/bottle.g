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
Include: 'franka.g'

Edit a0_base (table) {Q:<t(-.4 -.3 .05) d(90 0 0 1)>}

#goal (table_base){ type:ssBox, size:[0.2 0.2 0.01 .005], contact:0 Q:<[  -0.1, .0, -0.01, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}
#goalPose (table_base){ type:ssBox, size:[0.1 0.1 0.3 .005], contact:0 Q:<[  -0.1, .0, 0.25, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}
goalPose (table_base){ type:ssBox, size:[0.1 0.1 0.3 .005], contact:0 Q:<[  -0.1, .0, 0.15, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}

bottle (table_base){ joint:rigid type:cylinder, size:[0.001 0.001 0.3 .03], contact:1 Q:<[  -0.7, .175, 0.016, -1, 1, .0, 0]> color:[0.9, 0.9, 0.9, 0.5]}
#bottle (table_base){ joint:rigid type:cylinder, size:[0.001 0.001 0.3 .03], contact:1 Q:<[  -0.7, .375, 0.15, 1, 0, .0, 0]> color:[0.9, 0.9, 0.9, 0.5]}
bottle_base (bottle){ type:cylinder, size:[0.001 0.001 0.15 .03], contact:0 Q:<[  -0.0, .0, -0.075, 1, 0, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
bottle_cylinder (bottle_base){ type:capsule, size:[0.001 0.001 0.12 .03], contact:0 Q:<[  -0.0, .0, 0.04, 1, 0, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
bottle_neck (bottle_base){ type:cylinder, size:[0.1 0.06 0.25 .015], contact:0 Q:<[  -0.0, .0, 0.1, 1, 0, .0, 0]> color:[0.9, 0.1, 0.1, 1]}

bottle_1 (table_base){ type:cylinder, size:[0.1 0.06 0.25 .04], contact:1 Q:<[  -0.2, .0, 0.12, 1, 0, .0, .0]> color:[0.9, 0.9, 0.9, .7]}
bottle_2 (table_base){ type:cylinder, size:[0.1 0.06 0.25 .04], contact:1 Q:<[  -0.3, .0, 0.12, 1, 0, .0, .0]> color:[0.9, 0.9, 0.9, .7]}
bottle_3 (table_base){ type:cylinder, size:[0.1 0.06 0.25 .04], contact:1 Q:<[  -0.0, .0, 0.12, 1, 0, .0, .0]> color:[0.9, 0.9, 0.9, .7]}
bottle_4 (table_base){ type:cylinder, size:[0.1 0.06 0.25 .04], contact:1 Q:<[  0.1, .0, 0.12, 1, 0, .0, .0]> color:[0.9, 0.9, 0.9, .7]}

bottle_5 (table_base){ type:cylinder, size:[0.1 0.06 0.25 .04], contact:1 Q:<[  -0.2, .1, 0.12, 1, 0, .0, .0]> color:[0.9, 0.9, 0.9, .7]}
bottle_6 (table_base){ type:cylinder, size:[0.1 0.06 0.25 .04], contact:1 Q:<[  -0.3, .1, 0.12, 1, 0, .0, .0]> color:[0.9, 0.9, 0.9, .7]}
bottle_7 (table_base){ type:cylinder, size:[0.1 0.06 0.25 .04], contact:1 Q:<[  -0.0, .1, 0.12, 1, 0, .0, .0]> color:[0.9, 0.9, 0.9, .7]}
bottle_8 (table_base){ type:cylinder, size:[0.1 0.06 0.25 .04], contact:1 Q:<[  -0.1, .1, 0.12, 1, 0, .0, .0]> color:[0.9, 0.9, 0.9, .7]}
bottle_9 (table_base){ type:cylinder, size:[0.1 0.06 0.25 .04], contact:1 Q:<[  0.1, .1, 0.12, 1, 0, .0, .0]> color:[0.9, 0.9, 0.9, .7]}

bottle_10 (table_base){ type:cylinder, size:[0.1 0.06 0.25 .04], contact:1 Q:<[  -0.2, -.1, 0.12, 1, 0, .0, .0]> color:[0.9, 0.9, 0.9, .7]}
bottle_11 (table_base){ type:cylinder, size:[0.1 0.06 0.25 .04], contact:1 Q:<[  -0.3, -.1, 0.12, 1, 0, .0, .0]> color:[0.9, 0.9, 0.9, .7]}
bottle_12 (table_base){ type:cylinder, size:[0.1 0.06 0.25 .04], contact:1 Q:<[  -0.0, -.1, 0.12, 1, 0, .0, .0]> color:[0.9, 0.9, 0.9, .7]}
bottle_13 (table_base){ type:cylinder, size:[0.1 0.06 0.25 .04], contact:1 Q:<[  -0.1, -.1, 0.12, 1, 0, .0, .0]> color:[0.9, 0.9, 0.9, .7]}
bottle_14 (table_base){ type:cylinder, size:[0.1 0.06 0.25 .04], contact:1 Q:<[  0.1, -.1, 0.12, 1, 0, .0, .0]> color:[0.9, 0.9, 0.9, .7]}
