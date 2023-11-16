World 	{  X:<[0, 0, 0, 1, 0, 0, 0]> }                         
World_v2(World) 	{Q:<t(0 0. .2) d(0 0 0 1)>}     

Prefix: "a0_"
Include: 'mobile-manipulator-restricted.g'

Edit a0_base (World_v2) {Q:<t(1 -2. .2) d(90 0 0 1)>}
#Edit a0_base (World_v2) {Q:<t(-2 0. .2) d(90 0 0 1)>}

table (World){ joint:rigid type:ssBox, size:[1.25 0.75 0.1 .04], contact:1 Q:<[  -3, -2, 0.6, 1, 0, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
leg_1 (table){ type:cylinder, size:[0.1 0.1 0.4 .04], contact:1 Q:<[  0.6, -0.3, -0.2, 1, 0, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
leg_2 (table){ type:cylinder, size:[0.1 0.1 0.4 .04], contact:1 Q:<[  -0.6, -0.3, -0.2, 1, 0, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
leg_3 (table){ type:cylinder, size:[0.1 0.1 0.4 .04], contact:1 Q:<[  -0.6, 0.3, -0.2, 1, 0, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
leg_4 (table){ type:cylinder, size:[0.1 0.1 0.4 .04], contact:1 Q:<[  0.6, 0.3, -0.2, 1, 0, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
marker (table){ type:marker, size:[1], Q:<[  0, -0, 0., 1, 0, .0, 0]> color:[0.9, 0.1, 0.1, 1]}

goal (World){ type:ssBox, size:[1.25 0.75 0.1 .005], contact:0 Q:<[  3, -2, 0.6, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}
m (goal){ type:marker, size:[0.3], Q:<[  0, -0, 0., 1, 0, .0, 0]> color:[0.9, 0.1, 0.1, 1]}

_floor (World){  type:ssBox, size:[12 12 0.1 .005], contact:1 Q:<[ 0, 0, -0.05, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_obs_l (World){  type:ssBox, size:[0.2 6 2 .005], contact:1 Q:<[ 0, 4, 1, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_obs_r (World){  type:ssBox, size:[0.2 6 2 .005], contact:1 Q:<[ 0, -4, 1, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_obs_t (World){  type:ssBox, size:[0.2 2 0.5 .005], contact:1 Q:<[ 0, 0, 1.75, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}

