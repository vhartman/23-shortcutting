World 	{  X:<[0, 0, 0, 1, 0, 0, 0]> }                         

table_base (World) {
    Q:[0 0 .6]
    shape:marker, size:[.03],
}
table (table_base){
    shape:ssBox, Q:[0 0 -.1], size:[2 2 .05 .02], color:[.7 .7 .7]
    contact, logical:{ }
}

agent (table_base){ joint:transXY limits: [-1. 1 -1 1], type:cylinder, size:[1 1 0.1 0.15], contact:1 Q:<[  0.5, -.5, 0.0, 1, 0, .0, .0]> color:[0.1, 0.1, 0.9, 1]}

obj (table_base){ joint:rigid type:ssBox, size:[0.1 0.9 0.1 .005], contact:1 Q:<[  -0.2, -.4, 0.0, 1, 0, .0, .0]> color:[0.9, 0.1, 0.1, 1]}
goal (table_base){ type:ssBox, size:[0.3 0.3 0.1 .005], contact:0 Q:<[  0.6, .4, 0.0, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}

_obs_1 (table_base){  type:cylinder, size:[0.1 0.4 0.1 .05], contact:1 Q:<[  -0.05, -.7, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_obs_2 (table_base){  type:cylinder, size:[0.1 0.4 0.1 .05], contact:1 Q:<[  -0.05, -.1, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}

_obs_3 (table_base){  type:cylinder, size:[0.1 0.4 0.1 .05], contact:1 Q:<[  -0.35, -.7, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_obs_4 (table_base){  type:cylinder, size:[0.1 0.4 0.1 .05], contact:1 Q:<[  -0.35, -.1, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
