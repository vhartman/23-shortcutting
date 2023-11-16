World 	{  X:<[0, 0, 0, 1, 0, 0, 0]> }                         

table_base (World) {
    Q:[0 0 .6]
    shape:marker, size:[.03],
}
table (table_base){
    shape:ssBox, Q:[0 0 -.1], size:[3 2 .05 .02], color:[.7 .7 .7]
    contact, logical:{ }
}

agent (table_base){ joint:transXY limits: [-1.3 1.3 -0.8 0.8], type:cylinder, size:[0.6 0.6 0.1 0.15], contact:1 Q:<[  -0.5, .5, 0, 1, 0, .0, .0]> color:[0.1, 0.1, 0.9, 1]}
#agent (table_base){ joint:transXYPhi limits: [-1.3 1.3 -0.8 0.8, -4, 4], type:cylinder, size:[0.6 0.6 0.1 0.15], contact:1 Q:<[  -0.5, .5, 0, 1, 0, .0, .0]> color:[0.1, 0.1, 0.9, 1]}
goal (table_base){ type:ssBox, size:[0.3 0.3 0.1 .005], contact:0 Q:<[  -0.6, -.4, 0.0, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}
start (table_base){ type:ssBox, size:[0.3 0.3 0.1 .005], contact:0 Q:<[  0.6, -.4, 0.0, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}

obj (table_base){  type:ssBox, size:[0.1 0.1 0.1 .005], contact:1 Q:<[  0.0, 1.2, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}

_obs_top (table_base){  type:ssBox, size:[0.5 0.8 0.1 .005], contact:1 Q:<[  0.0, -.6, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_obs_bottom (table_base){  type:ssBox, size:[0.5 0.8 0.1 .005], contact:1 Q:<[  0.0, .6, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}

_top (table_base){  type:ssBox, size:[3 0.05 0.1 .005], contact:1 Q:<[  0.0, -1, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_bottom (table_base){  type:ssBox, size:[3 0.05 0.1 .005], contact:1 Q:<[  0.0, 1, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_left (table_base){  type:ssBox, size:[0.05 2 0.1 .005], contact:1 Q:<[  -1.5, -0, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_right (table_base){  type:ssBox, size:[0.05 2 0.1 .005], contact:1 Q:<[  1.5, -0, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
