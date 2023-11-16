World 	{  X:<[0, 0, 0, 1, 0, 0, 0]> }                         

table_base (World) {
    Q:[0 0 .6]
    shape:marker, size:[.03],
}
table (table_base){
    shape:ssBox, Q:[0 0 -.1], size:[3 2 .05 .02], color:[.7 .7 .7]
    contact, logical:{ }
}

agent (table_base){ joint:transXY limits: [-1.3 1.3 -0.8 0.8], type:cylinder, size:[0.6 0.6 0.1 0.15], contact:1 Q:<[  0, -.5, 0, 1, 0, .0, .0]> color:[0.1, 0.1, 0.9, 1]}
#agent (table_base){ joint:transXYPhi limits: [-1.3 1.3 -0.8 0.8, -4, 4], type:cylinder, size:[0.6 0.6 0.1 0.15], contact:1 Q:<[  0, -.5, 0, 1, 0, .0, .0]> color:[0.1, 0.1, 0.9, 1]}
start (table_base){ type:ssBox, size:[0.3 0.3 0.1 .005], contact:0 Q:<[  -0.8, -.7, 0.0, 1, 0, .0, .0]> color:[1, 0.1, 0.1, 0.3]}
goal1 (table_base){ type:ssBox, size:[0.3 0.3 0.1 .005], contact:0 Q:<[  1.0, .6, 0.0, 1, 0, .0, .0]> color:[0.4, 1, 1, 0.3]}
goal2 (table_base){ type:ssBox, size:[0.3 0.3 0.1 .005], contact:0 Q:<[  -0.2, .7, 0.0, 1, 0, .0, .0]> color:[0.1, 0.1, 1, 0.3]}
goal3 (table_base){ type:ssBox, size:[0.3 0.3 0.1 .005], contact:0 Q:<[  -1.2, 0.7, 0.0, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}

_obs_vert_right (table_base){  type:ssBox, size:[0.5 0.05 0.1 .005], contact:1 Q:<[ -0.75, -0.1, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_obs_vert_right_2 (table_base){  type:ssBox, size:[0.5 0.05 0.1 .005], contact:1 Q:<[ -0.25, -0.1, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_obs_vert_right_3 (table_base){  type:ssBox, size:[0.5 0.05 0.1 .005], contact:1 Q:<[ -0.25, 0.425, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}

_obs_top_left (table_base){  type:ssBox, size:[0.05 0.2 0.1 .005], contact:1 Q:<[  0.5, -0.9, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_obs_bottom_left (table_base){  type:ssBox, size:[0.05 1.4 0.1 .005], contact:1 Q:<[  0.5, .3, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}

_obs_top_right (table_base){  type:ssBox, size:[0.05 1.0 0.1 .005], contact:1 Q:<[  -0.5, -.5, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_obs_bottom_right (table_base){  type:ssBox, size:[0.05 0.6 0.1 .005], contact:1 Q:<[  -0.5, .7, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}

_top (table_base){  type:ssBox, size:[3 0.05 0.1 .005], contact:1 Q:<[  0.0, -1, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_bottom (table_base){  type:ssBox, size:[3 0.05 0.1 .005], contact:1 Q:<[  0.0, 1, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_left (table_base){  type:ssBox, size:[0.05 2 0.1 .005], contact:1 Q:<[  -1.5, -0, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_right (table_base){  type:ssBox, size:[0.05 2 0.1 .005], contact:1 Q:<[  1.5, -0, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
