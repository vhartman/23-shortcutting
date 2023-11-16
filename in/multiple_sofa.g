World 	{  X:<[0, 0, 0, 1, 0, 0, 0]> }                         

table_base (World) {
    Q:[0 0 .6]
    shape:marker, size:[.03],
}
table (table_base){
    shape:ssBox, Q:[0 0 -.1], size:[3 3 .05 .02], color:[.7 .7 .7]
    contact, logical:{ }
}

agent (table_base){ joint:transXYPhi limits: [-1.5 1.5 -1.5 1.5 -4 4], type:cylinder, size:[0.6 0.6 0.1 0.15], contact:1 Q:<[  -0.3, -.3, 0., 1, 0, .0, .0]> color:[0.1, 0.1, 0.9, 1]}

obj1 (table_base){ joint:rigid type:ssBox, size:[0.3 0.5 0.1 .005], contact:1 Q:<[  0.2, 0.2, 0.0, 1, 0, .0, .0]> color:[0.1, 1, 0.1, 1]}
obj2 (table_base){ joint:rigid type:ssBox, size:[0.3 0.6 0.1 .005], contact:1 Q:<[  1.0, 0.75, 0.0, 1, 0, .0, .0]> color:[0.9, 0.1, 0.1, 1]}

goal1 (table_base){ type:ssBox, size:[0.2 0.2 0.1 .005], contact:0 Q:<[  0.75, -.75, 0.0, 1, 0, .0, .0]> color:[0.1, 1, 0.1, 0.3]}
goal2 (table_base){ type:ssBox, size:[0.2 0.2 0.1 .005], contact:0 Q:<[  -0.75, -.75, 0.0, 1, 0, .0, .0]> color:[0.9, 0.1, 0.1, 0.3]}

_obs_wall_middle (table_base){  type:ssBox, size:[1.65 0.05 0.1 .005], contact:1 Q:<[  0.675, -0.2, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_obs_wall_middle_2 (table_base){  type:ssBox, size:[0.05 .5 0.1 .005], contact:1 Q:<[  -0.125, .05, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_obs_wall_middle3 (table_base){  type:ssBox, size:[0.8 0.8 0.1 .005], contact:1 Q:<[  -0.5, 0.5, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}

_obs_right_bound (table_base){  type:ssBox, size:[0.05 3 0.1 .005], contact:1 Q:<[  -1.475, .0, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_obs_left_bound (table_base){  type:ssBox, size:[0.05 3 0.1 .005], contact:1 Q:<[  1.475, .0, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_obs_bottom_bound (table_base){  type:ssBox, size:[3 0.05 0.1 .005], contact:1 Q:<[  0., 1.475, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_obs_top_bound (table_base){  type:ssBox, size:[3 0.05 0.1 .005], contact:1 Q:<[  0., -1.475, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
