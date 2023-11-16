World 	{  X:<[0, 0, 0, 1, 0, 0, 0]> }                         

table_base (World) {
    Q:[0 0 .6]
    shape:marker, size:[.03],
}
table (table_base){
    shape:ssBox, Q:[0 0 -.1], size:[3 3 .05 .02], color:[.7 .7 .7]
    contact, logical:{ }
}

agent (table_base){ joint:transXY limits: [-1.5 1.5 -1.5 1.5], type:cylinder, size:[0.6 0.6 0.1 0.15], contact:1 Q:<[  -0.5, .5, 0, 1, 0, .0, .0]> color:[0.1, 0.1, 0.9, 1]}
#agent (table_base){ joint:transXY limits: [-1.3 1.3 -0.8 0.8], type:cylinder, size:[0.6 0.6 0.1 0.1], contact:1 Q:<[  0.5, -.5, 0, 1, 0, .0, .0]> color:[0.1, 0.1, 0.9, 1]}
obj (table_base){ joint:rigid type:ssBox, size:[0.3 0.3 0.1 .005], contact:1 Q:<[  0.8, .4, 0.0, 1, 0, .0, .0]> color:[0.9, 0.1, 0.1, 1]}
goal (table_base){ type:ssBox, size:[0.3 0.3 0.1 .005], contact:0 Q:<[  -0.8, -.4, 0.0, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}

_obs_top (table_base){  type:ssBox, size:[0.5 0.6 0.1 .005], contact:1 Q:<[  0.0, -.5, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_obs_bottom (table_base){  type:ssBox, size:[0.5 1.3 0.1 .005], contact:1 Q:<[  0.0, 0.85, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}

_obs_right_bound (table_base){  type:ssBox, size:[0.05 3 0.1 .005], contact:1 Q:<[  -1.475, .0, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_obs_left_bound (table_base){  type:ssBox, size:[0.05 3 0.1 .005], contact:1 Q:<[  1.475, .0, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_obs_bottom_bound (table_base){  type:ssBox, size:[3 0.05 0.1 .005], contact:1 Q:<[  0., 1.475, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_obs_top_bound (table_base){  type:ssBox, size:[3 0.05 0.1 .005], contact:1 Q:<[  0., -1.475, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
