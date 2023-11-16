World 	{  X:<[0, 0, 0, 1, 0, 0, 0]> }                         

table_base (World) {
    Q:[0 0 .6]
    shape:marker, size:[.03],
}
table (table_base){
    shape:ssBox, Q:[0 0 -.05], size:[2.3 1.24 .05 .02], color:[.3 .3 .3]
    contact, logical:{ }
}

_obs (World) 	{  type:ssBox, size:[3 0.2 1 .005], contact:1 Q:<[  0.0, -0.7, 1.1, 1, 0, .0, .0]> color:[0, 0, 0, 0.1]}

Prefix: "a0_"
Include: 'franka.g'

Edit a0_base (table) {Q:<t(-.4 -.3 .05) d(90 0 0 1)>}

obj (table_base){ joint:rigid type:cylinder, size:[1 0.06 0.7 .01], contact:1 Q:<[  -0.4, .0, 0.05, 1, 0, 1, .0]> color:[0.9, 0.1, 0.1, 1]}

goal (table_base){ type:cylinder, size:[1 0.06 0.7 .01], contact:0 Q:<[  -0.4, .2, 0.05, 1, 0, 1, .0]> color:[0.0, 0.9, 0.1, 0.7]}

_obs_3 (table_base){  type:cylinder, size:[0.1 0.4 0.2 .02], contact:1 Q:<[  -0.15, -.05, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_obs_4 (table_base){  type:cylinder, size:[0.1 0.4 0.2 .02], contact:1 Q:<[  -0.15, .05, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_obs_4 (table_base){  type:cylinder, size:[0.1 0.4 0.1 .02], contact:1 Q:<[  -0.15, .0, 0.1, 1, 1, 0, .0]> color:[0.4, 0.4, 0.4, 1]}

_obs_3 (table_base){  type:cylinder, size:[0.1 0.4 0.2 .02], contact:1 Q:<[  -0.55, -.05, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_obs_4 (table_base){  type:cylinder, size:[0.1 0.4 0.2 .02], contact:1 Q:<[  -0.55, .05, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_obs_4 (table_base){  type:cylinder, size:[0.1 0.4 0.1 .02], contact:1 Q:<[  -0.55, .0, 0.1, 1, 1, 0, .0]> color:[0.4, 0.4, 0.4, 1]}
