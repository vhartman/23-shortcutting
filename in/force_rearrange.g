World 	{  X:<[0, 0, 0, 1, 0, 0, 0]> }                         
World_v2(World) 	{Q:<t(-0 0. .2) d(0 0 0 1)>}     

Prefix: "a0_"
Include: 'mobile-manipulator.g'

Edit a0_base (World_v2) {Q:<t(-2 0. .2) d(90 0 0 1)>}

stick (World){ joint:rigid type:cylinder, size:[0.001 0.001 2 .04], contact:1 Q:<[  -0.7, .3, 1.1, 1, 0, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
#stick (World){ joint:rigid type:cylinder, size:[0.001 0.001 2 .04], contact:1 Q:<[  -0.7, .3, 0.1, -1, 1, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
goal (World){ type:cylinder, size:[0.001 0.001 2 .04], contact:0 Q:<[  -0., .3, 0.75, -1, 1, .0, 0]> color:[0.0, 0.9, 0.1, 0.5]}

_obs_bottom (World){  type:ssBox, size:[1 1.2 0.7 .005], contact:1 Q:<[ 0, 0.3, 0.35, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_obs_top (World){  type:ssBox, size:[1 1.2 0.7 .005], contact:1 Q:<[ 0, 0.3, 1.5, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}

_floor (World){  type:ssBox, size:[12 12 0.1 .005], contact:1 Q:<[ 0, 0, -0.06, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
