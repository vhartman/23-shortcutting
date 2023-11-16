World 	{  X:<[0, 0, 0, 1, 0, 0, 0]> }                         
World_v2(World) 	{Q:<t(0 0. .2) d(0 0 0 1)>}     

Prefix: "a0_"
Include: 'mobile-manipulator.g'

Edit a0_base (World_v2) {Q:<t(1 0. .2) d(90 0 0 1)>}

podest (World){ joint:rigid type:ssBox, size:[0.9 0.5 3 .004], contact:1 Q:<[  -1, 0, 0.3, -1, 1, .0, 0]>}

obj1 (podest){ joint:rigid type:ssBox, size:[0.2 0.4 0.15 .004], contact:1 Q:<[  0.3, -0.4, 0, -1, 1, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
obj2 (podest){ joint:rigid type:ssBox, size:[0.2 0.4 0.15 .004], contact:1 Q:<[  0.3, -0.4, -0.5, -1, 1, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
obj3 (podest){ joint:rigid type:ssBox, size:[0.2 0.4 0.15 .004], contact:1 Q:<[  -0., -0.4, -1, -1, 1, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
obj4 (podest){ joint:rigid type:ssBox, size:[0.2 0.4 0.15 .004], contact:1 Q:<[  -0., -0.4, 1, -1, 1, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
obj5 (podest){ joint:rigid type:ssBox, size:[0.2 0.4 0.15 .004], contact:1 Q:<[  -0., -0.4, 0, -1, 1, .0, 0]> color:[0.9, 0.1, 0.1, 1]}

obj6 (podest){ joint:rigid type:ssBox, size:[0.2 0.4 0.15 .004], contact:1 Q:<[  0.3, -0.4, 0.5, -1, 1, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
obj7 (podest){ joint:rigid type:ssBox, size:[0.2 0.4 0.15 .004], contact:1 Q:<[  -0., -0.4, 0.5, -1, 1, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
obj8 (podest){ joint:rigid type:ssBox, size:[0.2 0.4 0.15 .004], contact:1 Q:<[  -0., -0.4, -0.5, -1, 1, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
obj9 (podest){ joint:rigid type:ssBox, size:[0.2 0.4 0.15 .004], contact:1 Q:<[  0.3, -0.4, 1, -1, 1, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
obj10 (podest){ joint:rigid type:ssBox, size:[0.2 0.4 0.15 .004], contact:1 Q:<[  0.3, -0.4, -1, -1, 1, .0, 0]> color:[0.9, 0.1, 0.1, 1]}

obj11 (podest){ joint:rigid type:ssBox, size:[0.2 0.4 0.15 .004], contact:1 Q:<[  -0.3, -0.4, 0.5, -1, 1, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
obj12 (podest){ joint:rigid type:ssBox, size:[0.2 0.4 0.15 .004], contact:1 Q:<[  -0.3, -0.4, 0.0, -1, 1, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
obj13 (podest){ joint:rigid type:ssBox, size:[0.2 0.4 0.15 .004], contact:1 Q:<[  -0.3, -0.4, -0.5, -1, 1, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
obj14 (podest){ joint:rigid type:ssBox, size:[0.2 0.4 0.15 .004], contact:1 Q:<[  -0.3, -0.4, 1, -1, 1, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
obj15 (podest){ joint:rigid type:ssBox, size:[0.2 0.4 0.15 .004], contact:1 Q:<[  -0.3, -0.4, -1, -1, 1, .0, 0]> color:[0.9, 0.1, 0.1, 1]}

obs1 (World){ type:ssBox, size:[0.2 0.4 0.15 .004], contact:1 Q:<[  2, 0.5, 0.25, 1, 0, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
obs2 (World){ type:ssBox, size:[0.2 0.4 0.15 .004], contact:1 Q:<[  2, -0.0, 0.25, 1, 0, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
obs3 (World){ type:ssBox, size:[0.2 0.4 0.15 .004], contact:1 Q:<[  2, -0.5, 0.25, 1, 0, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
obs4 (World){ type:ssBox, size:[0.2 0.4 0.15 .004], contact:1 Q:<[  2, 1, 0.25, 1, 0, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
obs5 (World){ type:ssBox, size:[0.2 0.4 0.15 .004], contact:1 Q:<[  2, -1, 0.25, 1, 0, .0, 0]> color:[0.9, 0.1, 0.1, 1]}

goal1 (World){ type:ssBox, size:[0.2 0.4 0.15 .005], contact:0 Q:<[  2, -0.5, 0.45, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}
goal2 (World){ type:ssBox, size:[0.2 0.4 0.15 .005], contact:0 Q:<[  2, 0, 0.45, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}
goal3 (World){ type:ssBox, size:[0.2 0.4 0.15 .005], contact:0 Q:<[  2, 0.5, 0.45, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}
goal4 (World){ type:ssBox, size:[0.2 0.4 0.15 .005], contact:0 Q:<[  2, -1, 0.45, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}
goal5 (World){ type:ssBox, size:[0.2 0.4 0.15 .005], contact:0 Q:<[  2, 1, 0.45, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}

goal6 (World){ type:ssBox, size:[0.2 0.4 0.15 .005], contact:0 Q:<[  2, -0.5, 0.65, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}
goal7 (World){ type:ssBox, size:[0.2 0.4 0.15 .005], contact:0 Q:<[  2, 0, 0.65, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}
goal8 (World){ type:ssBox, size:[0.2 0.4 0.15 .005], contact:0 Q:<[  2, 0.5, 0.65, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}
goal9 (World){ type:ssBox, size:[0.2 0.4 0.15 .005], contact:0 Q:<[  2, -1, 0.65, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}
goal10 (World){ type:ssBox, size:[0.2 0.4 0.15 .005], contact:0 Q:<[  2, 1, 0.65, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}

goal11 (World){ type:ssBox, size:[0.2 0.4 0.15 .005], contact:0 Q:<[  2, -0.5, 0.85, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}
goal12 (World){ type:ssBox, size:[0.2 0.4 0.15 .005], contact:0 Q:<[  2, 0, 0.85, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}
goal13 (World){ type:ssBox, size:[0.2 0.4 0.15 .005], contact:0 Q:<[  2, 0.5, 0.85, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}
goal14 (World){ type:ssBox, size:[0.2 0.4 0.15 .005], contact:0 Q:<[  2, -1, 0.85, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}
goal15 (World){ type:ssBox, size:[0.2 0.4 0.15 .005], contact:0 Q:<[  2, 1, 0.85, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}

_floor (World){  type:ssBox, size:[12 12 0.1 .005], contact:1 Q:<[ 0, 0, -0.05, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
#_obs_l (World){  type:ssBox, size:[0.2 6 2 .005], contact:1 Q:<[ 0, 3.5, 1, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
#_obs_r (World){  type:ssBox, size:[0.2 6 2 .005], contact:1 Q:<[ 0, -3.5, 1, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
#_obs_t (World){  type:ssBox, size:[0.2 1 0.5 .005], contact:1 Q:<[ 0, 0, 2., 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}

