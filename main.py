import matplotlib.pyplot as plt
from numpy import sin, cos, pi, array, linalg, set_printoptions

from controller import Supervisor, Motor

sup = Supervisor()

wheel0 = Motor('wheel0_joint')
wheel1 = Motor('wheel1_joint')
wheel2 = Motor('wheel2_joint')

wheel0.setVelocity(.0)
wheel1.setVelocity(.0)
wheel2.setVelocity(.0)

wheel0.setPosition(float('inf'))
wheel1.setPosition(float('inf'))
wheel2.setPosition(float('inf'))

time_step = 16.0 / 1000.0

x_values = []
y_values = []

robot_frame = array([.0, .0, .0])
target_frame = array([-5, -5, -pi / 4])

set_printoptions(precision=4, suppress=True)

robot = sup.getFromDef('ino')

lg = .0
g = .0

R = .45

_m = linalg.inv(array([
	[-cos(pi / 3), -cos(pi / 3), 1],
	[sin(pi / 3), -sin(pi / 3), 0],
	[1 / R, 1 / R, 1 / R]
]))

kx, ky, kg = 10, 10, 15

while sup.step(16) != -1:
	p = robot.getPosition()
	
	robot_frame[0] = p[0]
	robot_frame[1] = p[1]
	
	error = target_frame - robot_frame
	theta = robot_frame[2]
	
	err = array([
		[cos(theta), sin(theta), 0],
		[-sin(theta), cos(theta), 0],
		[0, 0, 1]
	]) @ error
	
	vd_x = kx * err[0]
	vd_y = ky * err[1]
	
	wd = (kx * err[0] + ky * err[1]) + kg * err[2]
	
	v = array([vd_x, vd_y, -wd])
	
	vb = _m @ v
	
	wheel0.setVelocity(vb[0])
	wheel1.setVelocity(vb[1])
	wheel2.setVelocity(vb[2])
	
	_v = robot.getVelocity()
	
	robot_frame[2] += _v[5] * time_step
	
	print(robot_frame, v)
	
	x_values.append(robot_frame[0])
	y_values.append(robot_frame[1])

fig, axs = plt.subplots(2, 1)

axs[0].plot(x_values, y_values, 'r')
plt.show()
