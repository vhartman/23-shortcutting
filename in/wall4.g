World 	{  X:<[0, 0, 0, 1, 0, 0, 0]> }                         
World_v2(World) 	{Q:<t(0 0. .2) d(0 0 0 1)>}     

Prefix: "a0_"
Include: 'mobile-manipulator.g'

Edit a0_base (World_v2) {Q:<t(1 0. .2) d(90 0 0 1)>}

podest (World){ joint:rigid type:ssBox, size:[0.6 0.5 2 .004], contact:1 Q:<[  -1, 0, 0.3, -1, 1, .0, 0]>}
obj1 (podest){ joint:rigid type:ssBox, size:[0.2 0.4 0.15 .004], contact:1 Q:<[  0.15, -0.4, 0, -1, 1, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
obj2 (podest){ joint:rigid type:ssBox, size:[0.2 0.4 0.15 .004], contact:1 Q:<[  0.15, -0.4, -0.5, -1, 1, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
obj4 (podest){ joint:rigid type:ssBox, size:[0.2 0.4 0.15 .004], contact:1 Q:<[  -0.15, -0.4, 0, -1, 1, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
obj3 (podest){ joint:rigid type:ssBox, size:[0.2 0.4 0.15 .004], contact:1 Q:<[  0.15, -0.4, 0.5, -1, 1, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
obj6 (podest){ joint:rigid type:ssBox, size:[0.2 0.4 0.15 .004], contact:1 Q:<[  -0.15, -0.4, 0.5, -1, 1, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
obj5 (podest){ joint:rigid type:ssBox, size:[0.2 0.4 0.15 .004], contact:1 Q:<[  -0.15, -0.4, -0.5, -1, 1, .0, 0]> color:[0.9, 0.1, 0.1, 1]}

obs1 (World){ type:ssBox, size:[0.2 0.4 0.15 .004], contact:1 Q:<[  2, 0.5, 0.25, 1, 0, .0, 0]> }
obs2 (World){ type:ssBox, size:[0.2 0.4 0.15 .004], contact:1 Q:<[  2, -0.0, 0.25, 1, 0, .0, 0]>}
obs3 (World){ type:ssBox, size:[0.2 0.4 0.15 .004], contact:1 Q:<[  2, -0.5, 0.25, 1, 0, .0, 0]> }
obs4 (World){ type:ssBox, size:[0.2 0.4 0.15 .004], contact:1 Q:<[  2, 1, 0.25, 1, 0, .0, 0]> }
obs5 (World){ type:ssBox, size:[0.2 0.4 0.15 .004], contact:1 Q:<[  2, -1, 0.25, 1, 0, .0, 0]> }

goal1 (World){ type:ssBox, size:[0.2 0.4 0.15 .005], contact:0 Q:<[  2, -0.5, 0.45, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}
goal2 (World){ type:ssBox, size:[0.2 0.4 0.15 .005], contact:0 Q:<[  2, 0, 0.45, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}
goal3 (World){ type:ssBox, size:[0.2 0.4 0.15 .005], contact:0 Q:<[  2, 0.5, 0.45, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}
goal4 (World){ type:ssBox, size:[0.2 0.4 0.15 .005], contact:0 Q:<[  2, -0.5, 0.65, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}
goal5 (World){ type:ssBox, size:[0.2 0.4 0.15 .005], contact:0 Q:<[  2, 0, 0.65, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}
goal6 (World){ type:ssBox, size:[0.2 0.4 0.15 .005], contact:0 Q:<[  2, 0.5, 0.65, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}

_floor (World){  type:ssBox, size:[12 12 0.1 .005], contact:1 Q:<[ 0, 0, -0.05, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
#_obs_l (World){  type:ssBox, size:[0.2 6 2 .005], contact:1 Q:<[ 0, 3.5, 1, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
#_obs_r (World){  type:ssBox, size:[0.2 6 2 .005], contact:1 Q:<[ 0, -3.5, 1, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
#_obs_t (World){  type:ssBox, size:[0.2 1 0.5 .005], contact:1 Q:<[ 0, 0, 2., 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}

