floor 	{ , shape:ssBox, size:[10, 10, 0.1, 0.01], contact:1 }
gripper_l (floor) 	{  Q:[0, 0, 0.1, -1, 0, 0, 0], shape:sphere, size:[1, 1, 1, 0.075], contact:1 }
a_0 (gripper_l) 	{  Q:[0.25, 0, 0, -1, 0, 0, 0], joint:rigid, shape:ssBox, size:[0.5, 0.1, 0.1, 0.01], contact:1 }
a_joint_0 (a_0) 	{  A:[0.25, 0, 0, -1, 0, 0, 0], joint:hingeX }
a_1 (a_joint_0) 	{  Q:[0.25, 0, 0, -1, 0, 0, 0], joint:rigid, shape:ssBox, size:[0.5, 0.1, 0.1, 0.01], contact:1 }
a_joint_1 (a_1) 	{  A:[0.25, 0, 0, -1, 0, 0, 0], joint:hingeX }
a_2 (a_joint_1) 	{  Q:[0.25, 0, 0, -1, 0, 0, 0], joint:rigid, shape:ssBox, size:[0.5, 0.1, 0.1, 0.01], contact:1 }
a_joint_2 (a_2) 	{  A:[0.25, 0, 0, -1, 0, 0, 0], joint:hingeX }
gripper_r (a_joint_2) 	{  A:[0.5, 0, 0, -1, 0, 0, 0], joint:hingeX, shape:sphere, size:[1, 1, 1, 0.075], contact:1 }
