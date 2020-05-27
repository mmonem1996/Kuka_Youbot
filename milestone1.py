import numpy as np 
import modern_robotics as mr


def NextState(robot_config, c_wheels_angles, control_speeds, dt, max_angular_speed, F_matrix):
	# robot_config = phi, x, y, th1, th2, th3, th4, th5
	c_chass_config = robot_config[0:3].reshape((3,1))
	c_arm_joints_val = robot_config[3:8].reshape((5,1))
	c_wheels_angles = np.array(c_wheels_angles).reshape((4,1))
	control_speeds  = np.array(control_speeds)
	F_matrix = np.array(F_matrix)
	# clap control speeds to the maximum permissible joint angle 
	if max_angular_speed != 0: 
		for i in range(9):
			control_speeds[i,0] = control_speeds[i,0] if (control_speeds[i,0] <=  max_angular_speed) else  max_angular_speed
			control_speeds[i,0] = control_speeds[i,0] if (control_speeds[i,0] >= -max_angular_speed) else -max_angular_speed
	# evaluating new chassis configuration via Odometry
	delta_wheels_angles = (control_speeds[0:4, 0] * dt).reshape((4,1))
	Vb = np.dot(F_matrix, delta_wheels_angles)
	delta_chass_config = np.zeros((3,1))
	wbz = Vb[0]
	vbx = Vb[1]
	vby = Vb[2]
	if wbz == 0:
		delta_chass_config[1,0] = vbx
		delta_chass_config[2,0] = vby
	else:
		delta_chass_config[0,0] = wbz
		delta_chass_config[1,0] = (vbx*np.sin(wbz) + vby*(np.cos(wbz) - 1))/wbz
		delta_chass_config[2,0] = (vby*np.sin(wbz) + vbx*(1 - np.cos(wbz)))/wbz

	thk = c_chass_config[0,0]

	R = np.array([[1, 0, 0], [0, np.cos(thk), -np.sin(thk)], [0, np.sin(thk), np.cos(thk)]])
	delta_chass_config = R @ delta_chass_config
	# integration via Euler step integration method
	new_wheel_angles = c_wheels_angles + delta_wheels_angles
	new_arm_joints_values = c_arm_joints_val + control_speeds[4:9, 0].reshape((5,1)) * dt
	# new_arm_joints_values, _ = mr.EulerStep(c_arm_joints_val, control_speeds[4:9, 0].reshape((5,1)), np.zeros((5,1)), dt)
	new_chassis_config = c_chass_config + delta_chass_config
	new_robot_config = np.vstack((new_chassis_config, new_arm_joints_values))

	return new_robot_config.reshape((8,)), new_wheel_angles.reshape((4,))