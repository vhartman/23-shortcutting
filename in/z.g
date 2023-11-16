World 	{  }
a0_world 	{  X:[0, 0, 0.2, 1, 0, 0, 0] }
World_v2 (World) 	{  Q:[-2, 0, 0.2, 1, 0, 0, 0] }
stick (World) 	{  Q:[-3, 0, 0.1, -0.707107, 0.707107, 0, 0], joint:rigid, shape:cylinder, size:[0.001, 0.001, 1.6, 0.04], color:[0.9, 0.1, 0.1, 1], contact:1 }
goal (World) 	{  Q:[3, 2, 0.05, 1, 0, 0, 0], shape:ssBox, size:[2, 2, 0.1, 0.005], color:[0.4, 1, 0.4, 0.3] }
_floor (World) 	{  Q:[0, 0, -0.05, 1, 0, 0, 0], shape:ssBox, size:[12, 12, 0.1, 0.005], color:[0.4, 0.4, 0.4, 1], contact:1 }
_obs_l (World) 	{  Q:[0, 3.5, 1, 1, 0, 0, 0], shape:ssBox, size:[0.2, 6, 2, 0.005], color:[0.4, 0.4, 0.4, 1], contact:1 }
_obs_r (World) 	{  Q:[0, -3.5, 1, 1, 0, 0, 0], shape:ssBox, size:[0.2, 6, 2, 0.005], color:[0.4, 0.4, 0.4, 1], contact:1 }
_obs_t (World) 	{  Q:[0, 0, 1.75, 1, 0, 0, 0], shape:ssBox, size:[0.2, 1, 0.5, 0.005], color:[0.4, 0.4, 0.4, 1], contact:1 }
a0_base (World_v2) 	{  Q:[-2, 0, 0.2, 0.707107, 0, 0, 0.707107], joint:transXY, limits:[-6, 6, -6, 6], shape:ssBox, size:[0.35, 0.35, 0.4, 0.05], color:[1, 0.5, 0.5, 1] }
m (stick) 	{ , shape:marker, size:[1], color:[0.9, 0.1, 0.1, 1] }
m (goal) 	{ , shape:marker, size:[1], color:[0.9, 0.1, 0.1, 1] }
a0_base_coll (a0_base) 	{ , shape:ssBox, size:[0.4, 0.4, 0.4, 0.05], color:[1, 1, 1, 0.1], contact:1 }
a0_rot (a0_base) 	{ , joint:hingeZ, limits:[-4, 4] }
a0_arm0 (a0_rot) 	{  Q:[0, 0, 0.3, 1, 0, 0, 0], shape:capsule, size:[0.4, 0.08] }
a0_arm0_col (a0_rot) 	{ , shape:cylinder, size:[0.5, 0.1], color:[0.6, 0.6, 0.6, 1] }
a0_arm0_coll (a0_arm0) 	{ , shape:capsule, size:[0.2, 0.1], color:[1, 1, 1, 0.2], contact:-2 }
a0_arm0>a0_joint1 (a0_arm0) 	{  Q:[0, 0, 0.2, 1, 0, 0, 0] }
a0_joint1 (a0_arm0>a0_joint1) 	{  Q:[0, 0, 0, 0.877583, 0.479426, 0, 0], joint:hingeX, limits:[-1.7, 1.7] }
a0_cyl_col (a0_joint1) 	{  Q:[0, 0, 0, 0.707107, 0, 0.707063, 0.00785626], shape:cylinder, size:[0.18, 0.07], color:[1, 0.5, 0.5, 1] }
a0_cyl_noncol (a0_joint1) 	{  Q:[0, 0, 0, 0.707107, 0, 0.707063, 0.00785626], shape:cylinder, size:[0.17, 0.081], color:[0.6, 0.6, 0.6, 1] }
a0_arm1 (a0_joint1) 	{  Q:[0, 0, 0.4, 1, 0, 0, 0], shape:capsule, size:[0.8, 0.06] }
a0_arm1_coll (a0_arm1) 	{ , shape:capsule, size:[0.5, 0.08], color:[1, 1, 1, 0.2], contact:-2 }
a0_arm1>a0_joint2 (a0_arm1) 	{  Q:[0, 0, 0.4, 1, 0, 0, 0] }
a0_joint2 (a0_arm1>a0_joint2) 	{  Q:[0, 0, 0, 0.877583, 0.479426, 0, 0], joint:hingeX, limits:[-1.7, 1.7] }
a0_cyl2_col (a0_joint2) 	{  Q:[0, 0, 0, 0.707107, 0, 0.707063, 0.00785626], shape:cylinder, size:[0.141, 0.06], color:[1, 0.5, 0.5, 1] }
a0_cyl2_noncol (a0_joint2) 	{  Q:[0, 0, 0, 0.707107, 0, 0.707063, 0.00785626], shape:cylinder, size:[0.14, 0.066], color:[0.6, 0.6, 0.6, 1] }
a0_arm2 (a0_joint2) 	{  Q:[0, 0, 0.2, 1, 0, 0, 0], shape:capsule, size:[0.4, 0.04] }
a0_arm2_coll (a0_arm2) 	{ , shape:capsule, size:[0.3, 0.06], color:[1, 1, 1, 0.2], contact:1 }
a0_arm2>a0_joint2a (a0_arm2) 	{  Q:[0, 0, 0.25, 1, 0, 0, 0] }
a0_joint2a (a0_arm2>a0_joint2a) 	{ , joint:hingeZ, limits:[-2, 2] }
a0_joint2b (a0_joint2a) 	{ , joint:hingeX, limits:[-2, 2] }
a0_joint2c (a0_joint2b) 	{ , joint:hingeY, limits:[-2, 2] }
a0_gripper (a0_joint2c) 	{  Q:[0, 0, 0.01, 6.12323e-17, 1, 0, 0], shape:sphere, size:[0.03], color:[1, 0.5, 0.5, 1] }

