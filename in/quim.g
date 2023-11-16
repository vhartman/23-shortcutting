World 	{  X:<[0, 0, 0, 1, 0, 0, 0]> }                         

table_base (World) {
    Q:[0 0 .6]
    shape:marker, size:[.03],
}
table (table_base){
    shape:ssBox, Q:[0 0 -.1], size:[2 2 .05 .02], color:[.9 .9 .9]
    contact, logical:{ }
}

agent (table_base){ joint:transXYPhi limits: [-1 1. -1 1 -4 4], type:cylinder, size:[0.6 0.6 0.1 0.1], contact:1 Q:<[  0.5, -.5, 0., 1, 0, .0, .0]> color:[0.1, 0.1, 0.9, 1]}
#agent (table_base){ joint:transXY limits: [-1 1. -1 1], type:cylinder, size:[0.6 0.6 0.1 0.1], contact:1 Q:<[  0.5, -.5, 0., 1, 0, .0, .0]> color:[0.1, 0.1, 0.9, 1]}

obj1 (table_base){ joint:rigid type:ssBox, size:[0.3 0.3 0.1 .005], contact:1 Q:<[  0.6, .5, 0.0, 1, 0, .0, .0]> color:[0.6, 0.1, 0.9, 1]}
obj2 (table_base){ joint:rigid type:ssBox, size:[0.3 0.3 0.1 .005], contact:1 Q:<[  0.0, -0.5, 0.0, 1, 0, .0, .0]> color:[0.9, 0.1, 0.1, 1]}

#goal (table_base){ type:ssBox, size:[0.3 0.6 0.1 .005], contact:0 Q:<[  -0.025, .55, 0.0, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}
goal (table_base){ type:ssBox, size:[0.3 0.6 0.1 .005], contact:0 Q:<[  -0.75, .55, 0.0, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}
#goal (table_base){ type:ssBox, size:[0.3 2 0.1 .005], contact:0 Q:<[  -0.75, .0, 0.0, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}

#_obs_misc (table_base){  type:ssBox, size:[0.1 0.9 0.1 .005], contact:1 Q:<[  0.2, .2, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}

#_obs_side (table_base){  type:ssBox, size:[0.1 0.9 0.1 .005], contact:1 Q:<[  0.2, .55, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
#_obs_back (table_base){  type:ssBox, size:[0.1 0.9 0.1 .005], contact:1 Q:<[  -0.275, .55, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_obs_top (table_base){  type:ssBox, size:[0.1 0.5 0.1 .005], contact:1 Q:<[  -0.5, -.75, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_obs_side (table_base){  type:ssBox, size:[0.1 0.9 0.1 .005], contact:1 Q:<[  -0.5, .55, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
#_obs_back (table_base){  type:ssBox, size:[0.1 0.9 0.1 .005], contact:1 Q:<[  -0.975, .55, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
#_obs_back (table_base){  type:ssBox, size:[0.1 2 0.1 .005], contact:1 Q:<[  -0.975, .0, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
#_obs_back2 (table_base){  type:ssBox, size:[0.5 0.1 0.1 .005], contact:1 Q:<[  -.75, .975, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
#_obs_back3 (table_base){  type:ssBox, size:[0.5 0.1 0.1 .005], contact:1 Q:<[  -.75, -.975, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}

_top (table_base){  type:ssBox, size:[2 0.05 0.1 .005], contact:1 Q:<[  0.0, -1, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_bottom (table_base){  type:ssBox, size:[2 0.05 0.1 .005], contact:1 Q:<[  0.0, 1, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_left (table_base){  type:ssBox, size:[0.05 2 0.1 .005], contact:1 Q:<[  -1., -0, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
_right (table_base){  type:ssBox, size:[0.05 2 0.1 .005], contact:1 Q:<[  1., -0, 0.0, 1, 0, .0, .0]> color:[0.4, 0.4, 0.4, 1]}
