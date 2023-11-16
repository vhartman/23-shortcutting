World 	{  X:<[0, 0, 0, 1, 0, 0, 0]> }                         
World_v2(World) 	{Q:<t(-2 0. .2) d(0 0 0 1)>}     

Prefix: "a0_"
Include: 'mobile-manipulator.g'

Edit a0_base (World_v2) {Q:<t(5 0. .2) d(0 0 0 1)>}
#Edit a0_base (World_v2) {Q:<t(-2 0. .2) d(90 0 0 1)>}

table (World){ joint:rigid type:ssBox, size:[1.2 0.6 0.1 .04], contact:1 Q:<[  -3, -2, 0.6, 1, 0, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
leg_1 (table){ type:cylinder, size:[0.1 0.1 0.4 .04], contact:1 Q:<[  0.6, -0.3, -0.2, 1, 0, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
leg_2 (table){ type:cylinder, size:[0.1 0.1 0.4 .04], contact:1 Q:<[  -0.6, -0.3, -0.2, 1, 0, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
leg_3 (table){ type:cylinder, size:[0.1 0.1 0.4 .04], contact:1 Q:<[  -0.6, 0.3, -0.2, 1, 0, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
leg_4 (table){ type:cylinder, size:[0.1 0.1 0.4 .04], contact:1 Q:<[  0.6, 0.3, -0.2, 1, 0, .0, 0]> color:[0.9, 0.1, 0.1, 1]}
marker (table){ type:marker, size:[1], Q:<[  0, -0, 0., 1, 0, .0, 0]> color:[0.9, 0.1, 0.1, 1]}

goal (World){ type:ssBox, size:[1.25 0.75 0.1 .005], contact:0 Q:<[  2, 0, 0.6, 1, 0, .0, 1]> color:[0.4, 1, 0.4, 0.3]}
m (goal){ type:marker, size:[0.3], Q:<[  0, -0, 0., 1, 0, .0, 0]> color:[0.9, 0.1, 0.1, 1]}

_cupboard_1 (World){  type:ssBox, size:[.6 .6 1.3 .005], contact:1 Q:<[ -1.5, 0.9, 0.65, 1, 0, .0, 1]> color:[0.4, 0.4, 0.4, 1]}
_cupboard_2 (World){  type:ssBox, size:[.6 .6 1.3 .005], contact:1 Q:<[ -2.5, 0.9, 0.65, 1, 0, .0, 1]> color:[0.4, 0.4, 0.4, 1]}
_img (World){  type:ssBox, size:[1 .1 .7 .005], contact:1 Q:<[ 0.2, 0.5, 0.85, 1, 0, .0, 1]> color:[0.6, 0.7, 0.4, 1]}
_img2 (World){  type:ssBox, size:[.1 1 .7 .005], contact:1 Q:<[ -3, -4.3, 0.85, 1, 0, .0, 1]> color:[0.6, 0.2, 0.0, 1]}

_obs1 (World){  type:ssBox, size:[0.2 7 2 .005], contact:1 Q:<[ -3.5, 1.5, 1, 1, 0, .0, 1]> color:[0.4, 0.4, 0.4, 1]}
_obs2 (World){  type:ssBox, size:[0.2 7 2 .005], contact:1 Q:<[ -3.5, -4.5, 1, 1, 0, .0, 1]> color:[0.4, 0.4, 0.4, 1]}

_obs3 (World){  type:ssBox, size:[0.2 3 2 .005], contact:1 Q:<[ 1.5, -3.75, 1, 1, 0, .0, 1]> color:[0.4, 0.4, 0.4, 1]}
_obs4 (World){  type:ssBox, size:[0.2 1 2 .005], contact:1 Q:<[ 0.5, -2.1, 1, 1, 0, .0, 1]> color:[0.4, 0.4, 0.4, 1]}
_obs5 (World){  type:ssBox, size:[0.2 3 2 .005], contact:1 Q:<[ 4, -2.1, 1, 1, 0, .0, 1]> color:[0.4, 0.4, 0.4, 1]}
#_obs5 (World){  type:ssBox, size:[0.2 3 2 .005], contact:1 Q:<[ 3.5, -2.1, 1, 1, 0, .0, 1]> color:[0.4, 0.4, 0.4, 1]}
#_obs6_t (World){  type:ssBox, size:[0.2 1.5 0.5 .005], contact:1 Q:<[ 1.6, -2.1, 1.75, 1, 0, .0, 1]> color:[0.4, 0.4, 0.4, 1]}

_obs_l (World){  type:ssBox, size:[0.2 6 2 .005], contact:1 Q:<[ 0, 0.7, 1, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_obs_r (World){  type:ssBox, size:[0.2 6 2 .005], contact:1 Q:<[ 0, -6.7, 1, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
#_obs_t (World){  type:ssBox, size:[0.2 1 0.5 .005], contact:1 Q:<[ 0, -3, 1.75, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_obs_back (World){  type:ssBox, size:[0.2 6 2 .005], contact:1 Q:<[ -6, -1.5, 1, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}

_floor (World){  type:ssBox, size:[12 12 0.1 .005], contact:1 Q:<[ 0, 0, -0.06, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
