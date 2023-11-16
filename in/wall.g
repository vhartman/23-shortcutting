World 	{  X:<[0, 0, 0, 1, 0, 0, 0]> }                         
World_v2(World) 	{Q:<t(4 0. .2) d(0 0 0 1)>}     

Prefix: "a0_"
Include: 'mobile-manipulator.g'

Edit a0_base (World_v2) {Q:<t(-2 0. .2) d(90 0 0 1)>}

obj1 (World){ joint:rigid type:ssBox, size:[0.5 0.5 0.5 .004], contact:1 Q:<[  -3, -0, 0.3, -1, 1, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
obj2 (World){ joint:rigid type:ssBox, size:[0.5 0.5 0.5 .004], contact:1 Q:<[  -3, -1, 0.3, -1, 1, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
obj3 (World){ joint:rigid type:ssBox, size:[0.5 0.5 0.5 .004], contact:1 Q:<[  -2, -0, 0.3, -1, 1, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
obj4 (World){ joint:rigid type:ssBox, size:[0.5 0.5 0.5 .004], contact:1 Q:<[  -2, -1, 0.3, -1, 1, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
obj5 (World){ joint:rigid type:ssBox, size:[0.5 0.5 0.5 .004], contact:1 Q:<[  -3, 1, 0.3, -1, 1, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
obj6 (World){ joint:rigid type:ssBox, size:[0.5 0.5 0.5 .004], contact:1 Q:<[  -2, 1, 0.3, -1, 1, .0, 0]> color:[0.9, 0.1, 0.1, 1]}

goal1 (World){ type:ssBox, size:[0.4 0.4 0.4 .005], contact:0 Q:<[  3, 2, 0.25, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}
goal2 (World){ type:ssBox, size:[0.4 0.4 0.4 .005], contact:0 Q:<[  3, 1.5, 0.25, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}
goal3 (World){ type:ssBox, size:[0.4 0.4 0.4 .005], contact:0 Q:<[  3, 1, 0.25, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}
goal4 (World){ type:ssBox, size:[0.4 0.4 0.4 .005], contact:0 Q:<[  3, 1.75, 0.68, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}
goal5 (World){ type:ssBox, size:[0.4 0.4 0.4 .005], contact:0 Q:<[  3, 1.25, 0.68, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}
goal6 (World){ type:ssBox, size:[0.4 0.4 0.4 .005], contact:0 Q:<[  3, 1.5, 1.1, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}

_floor (World){  type:ssBox, size:[12 12 0.1 .005], contact:1 Q:<[ 0, 0, -0.05, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
#_obs_l (World){  type:ssBox, size:[0.2 6 2 .005], contact:1 Q:<[ 0, 3.5, 1, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
#_obs_r (World){  type:ssBox, size:[0.2 6 2 .005], contact:1 Q:<[ 0, -3.5, 1, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
#_obs_t (World){  type:ssBox, size:[0.2 1 0.5 .005], contact:1 Q:<[ 0, 0, 2., 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}

