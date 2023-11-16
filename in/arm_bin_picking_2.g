World 	{  X:<[0, 0, 0, 1, 0, 0, 0]> }                         

table_base (World) {
    Q:[0 0 .6]
    shape:marker, size:[.03],
}
table (table_base){
    shape:ssBox, Q:[0 0 -.05], size:[2.3 1.24 .05 .02], color:[.3 .3 .3]
    contact:1, logical:{ }
}

_obs (World) 	{  type:ssBox, size:[3 0.2 1 .005], contact:1 Q:<[  0.0, -0.7, 1.1, 1, 0, .0, .0]> color:[0, 0, 0, 0.1]}

Prefix: "a0_"
Include: 'franka.g'
#Include: 'franka_touch.g'

Edit a0_base (table) {Q:<t(-.4 -.3 .05) d(90 0 0 1)>}

goal1 (table_base){ type:ssBox, size:[0.3 0.3 0.01 .005], contact:0 Q:<[  0.1, -0.1, 0.0, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}
goal2 (table_base){ type:ssBox, size:[0.3 0.3 0.01 .005], contact:0 Q:<[  -0.4, .1, -0.0, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.3]}
goal3 (table_base){ type:ssBox, size:[0.3 0.3 0.01 .005], contact:0 Q:<[  -0.9, -0.1, -0.0, 1, 0, .0, .0]> color:[0.4, 1, 0.4, 0.0]}

obj1 (table_base){ joint:rigid type:ssBox, size:[0.06 0.06 0.06 .01], contact:1 Q:<[  -0.8, -0.15, 0.03, 1, 0, .0, 0]> color:[1, 0, 0, 1]}
obj2 (table_base){ joint:rigid type:ssBox, size:[0.06 0.06 0.06 .01], contact:1 Q:<[  -0.9, 0., 0.03, 1, 0, .0, 0]> color:[1, 1, 0, 1]}
obj3 (table_base){ joint:rigid type:ssBox, size:[0.06 0.06 0.06 .01], contact:1 Q:<[  -1, -0.2, 0.03, 1, 0, .0, 0]> color:[1, 0, 1, 1]}

_wall1 (goal1){ type:ssBox, size:[0.025 0.45 0.15 .001], contact:1 Q:<[  -0.225, .0, 0.05, 1, 0, .0, 0]> color:[0.9, 0.9, 0.9, 1]}
_wall2 (goal1){ type:ssBox, size:[0.025 0.45 0.15 .001], contact:1 Q:<[  0.225, .0, 0.05, 1, 0, .0, 0]> color:[0.9, 0.9, 0.9, 1]}
_wall3 (goal1){ type:ssBox, size:[0.45 0.025 0.15 .001], contact:1 Q:<[  0.0, .225, 0.05, 1, 0, .0, 0]> color:[0.9, 0.9, 0.9, 1]}
_wall4 (goal1){ type:ssBox, size:[0.45 0.025 0.15 .001], contact:1 Q:<[  0.0, -.225, 0.05, 1, 0, .0, 0]> color:[0.9, 0.9, 0.9, 1]}

_wall5 (goal2){ type:ssBox, size:[0.025 0.45 0.15 .001], contact:1 Q:<[  -0.225, .0, 0.05, 1, 0, .0, 0]> color:[0.9, 0.9, 0.9, 1]}
_wall6 (goal2){ type:ssBox, size:[0.025 0.45 0.15 .001], contact:1 Q:<[  0.225, .0, 0.05, 1, 0, .0, 0]> color:[0.9, 0.9, 0.9, 1]}
_wall7 (goal2){ type:ssBox, size:[0.45 0.025 0.15 .001], contact:1 Q:<[  0.0, .225, 0.05, 1, 0, .0, 0]> color:[0.9, 0.9, 0.9, 1]}
_wall8 (goal2){ type:ssBox, size:[0.45 0.025 0.15 .001], contact:1 Q:<[  0.0, -.225, 0.05, 1, 0, .0, 0]> color:[0.9, 0.9, 0.9, 1]}

_wall9 (goal3){ type:ssBox, size:[0.025 0.45 0.15 .001], contact:1 Q:<[  -0.225, .0, 0.05, 1, 0, .0, 0]> color:[0.9, 0.9, 0.9, 1]}
_wall10 (goal3){ type:ssBox, size:[0.025 0.45 0.15 .001], contact:1 Q:<[  0.225, .0, 0.05, 1, 0, .0, 0]> color:[0.9, 0.9, 0.9, 1]}
_wall11 (goal3){ type:ssBox, size:[0.45 0.025 0.15 .001], contact:1 Q:<[  0.0, .225, 0.05, 1, 0, .0, 0]> color:[0.9, 0.9, 0.9, 1]}
_wall12 (goal3){ type:ssBox, size:[0.45 0.025 0.15 .001], contact:1 Q:<[  0.0, -.225, 0.05, 1, 0, .0, 0]> color:[0.9, 0.9, 0.9, 1]}
