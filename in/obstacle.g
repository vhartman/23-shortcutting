World 	{  X:<[0, 0, 0, 1, 0, 0, 0]> }                         

table_base (World) {
    Q:[0 0 .6]
    shape:marker, size:[.03],
}
table (table_base){
    shape:ssBox, Q:[0 0 -.1], size:[3 2 .05 .02], color:[.7 .7 .7]
    contact, logical:{ }
}

agent (table_base){ joint:transXY limits: [-1.4 1.4 -0.9 0.9], type:cylinder, size:[0.6 0.6 0.1 0.15], contact:1 Q:<[  1, .5, 0., 1, 0, .0, .0]> color:[0.1, 0.1, 0.9, 1]}
#agent (table_base){ joint:transXYPhi limits: [-1.4 1.4 -0.9 0.9 -4 4], type:cylinder, size:[0.6 0.6 0.1 0.15], contact:1 Q:<[  1, .5, 0., 1, 0, .0, .0]> color:[0.1, 0.1, 0.9, 1]}
obj (table_base){ joint:rigid type:ssBox, size:[0.4 0.6 0.1 .005], contact:1 Q:<[  0.25, 0., 0.0, 1, 0, .0, .0]> color:[0.9, 0.1, 0.1, 1]}
goal (table_base){ joint:rigid type:ssBox, size:[0.2 0.2 0.1 .005], contact:0 Q:<[  -0.75, .4, 0.0, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}

_obs_middle_left (table_base){  type:ssBox, size:[0.05 0.8 0.1 .005], contact:1 Q:<[  0, .6, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_obs_middle_right (table_base){  type:ssBox, size:[0.05 0.8 0.1 .005], contact:1 Q:<[  0, -.6, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}

#_obs_fixture_1 (table_base){  type:ssBox, size:[0.05 0.8 0.1 .005], contact:1 Q:<[  0.5, -.6, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
#_obs_fixture_2 (table_base){  type:ssBox, size:[0.2 0.05 0.1 .005], contact:1 Q:<[  0.1, .4, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}

_obs_right_bound (table_base){  type:ssBox, size:[0.05 2 0.1 .005], contact:1 Q:<[  -1.475, .0, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_obs_left_bound (table_base){  type:ssBox, size:[0.05 2 0.1 .005], contact:1 Q:<[  1.475, .0, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_obs_bottom_bound (table_base){  type:ssBox, size:[3 0.05 0.1 .005], contact:1 Q:<[  0., 0.975, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_obs_top_bound (table_base){  type:ssBox, size:[3 0.05 0.1 .005], contact:1 Q:<[  0., -0.975, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
