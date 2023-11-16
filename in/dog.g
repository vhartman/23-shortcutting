World 	{  X:<[0, 0, 0, 1, 0, 0, 0]> }                         
World_v2(World) 	{Q:<t(-2 0. .2) d(0 0 0 1)>}     

Prefix: "a0_"
Include: 'mobile-manipulator.g'

Edit a0_base (World_v2) {Q:<t(-2 0. .2) d(90 0 0 1)>}

stick (World){ joint:rigid type:cylinder, size:[0.001 0.001 1.6 .04], contact:1 Q:<[  -3, -0, 0.1, -1, 1, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
stick_extension (stick){ type:cylinder, size:[0.001 0.001 2 .04], contact:1 Q:<[  0, -0, 0., 1, 0, .0, 0]> color:[0.9, 0.9, 0.1, 1]}
m (stick){ type:marker, size:[1], Q:<[  0, -0, 0., 1, 0, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
goal (World){ type:ssBox, size:[2 2 0.1 .005], contact:0 Q:<[  3, 2, 0.05, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}
m (goal){ type:marker, size:[1], Q:<[  0, -0, 0., 1, 0, .0, 0]> color:[0.9, 0.1, 0.1, 1]}

_floor (World){  type:ssBox, size:[12 12 0.1 .005], contact:1 Q:<[ 0, 0, -0.05, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_obs_l (World){  type:ssBox, size:[0.2 6 2 .005], contact:1 Q:<[ 0, 3.5, 1, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_obs_r (World){  type:ssBox, size:[0.2 6 2 .005], contact:1 Q:<[ 0, -3.5, 1, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_obs_t (World){  type:ssBox, size:[0.2 1 0.5 .005], contact:1 Q:<[ 0, 0, 2., 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}

