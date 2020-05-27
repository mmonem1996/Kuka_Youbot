import numpy as np
from milestone1 import NextState
from milestone2 import TrajectoryGenerator
from milestone3 import testJointsLimits, Fkin, get_jacobian, calculate_control_speed, calculate_desired_twist, reset_integrator
import csv


F_Mat = [[-0.030844,0.030844,0.030844,-0.030844],[0.011875,0.011875,0.011875,0.011875],[-0.011875,0.011875,-0.011875,0.011875]]
M0e = [[1, 0, 0, 0.033], [0, 1, 0, 0], [0, 0, 1, 0.6546], [0, 0, 0, 1]]
Tb0 = [[1, 0, 0, 0.1662], [0, 1, 0, 0], [0, 0, 1, 0.0026], [0, 0, 0, 1]]
Arm_Blist = [[0,0,0,0,0],[0,-1,-1,-1,0],[1,0,0,0,1],[0,-0.5076,-0.3526,-0.2176,0],[0.033,0,0,0,0],[0,0,0,0,0]]


config = np.array([0.6, -0.5, 0.5, 0, -0.3, 0.2, -1.6, 0])
Tse = [[0, 0, 1, 0], [0, 1, 0, 0],[-1, 0, 0, 0.5],[0, 0, 0, 1]]
Tscinitial = [[1, 0, 0, 1], [0, 1, 0, 0], [0, 0, 1, 0.025], [0, 0, 0, 1]]
Tscgoal = [[0, 1, 0, 0], [-1, 0, 0, -1], [0, 0, 1, 0.025], [0, 0, 0, 1]]
Tcegrasp = [[0, 0, 1, 0], [0, 1, 0, 0],[-1, 0, 0, 0],[0, 0, 0, 1]]
Tcestand = [[0, 0, 1, 0], [0, 1, 0, 0],[-1, 0, 0, 0.4],[0, 0, 0, 1]]
wheels_angles = np.array([0, 0, 0, 0])


k_per_centi_sec = 2
print('Generating Trajectory for the main task...')
Traj = TrajectoryGenerator(Tse, Tscinitial, Tscgoal, Tcestand, Tcegrasp, 3, 4, k_per_centi_sec)
print('Generating CSV files for the main task (best)...')

dt = 0.01 / k_per_centi_sec
Kp = 5
Ki = 1
reset_integrator()
# this is to switch the feedward on or off
Kd = 1
t = 0
csvfile = open('../results/best/best.csv', 'w')
csvxerr = open('../results/best/best_xerrlog.csv', 'w')
csvobject = csv.writer(csvfile)
csvXerrObject = csv.writer(csvxerr)
csvobject.writerow(['#phi', 'x', 'y', 'Arm_th1', 'Arm_th2', 'Arm_th3', 'Arm_th4', 'Arm_th5', 'wheel1', 'wheel2', 'wheel3', 'wheel4'])
csvXerrObject.writerow(['t', 'wx', 'wy', 'wz', 'x', 'y', 'z'])
for i in range(len(Traj) - 1):
	state = Traj[i]
	Td = state[0]
	Td_next = Traj[i + 1][0]

	T = Fkin(config, Arm_Blist, M0e, Tb0)
	V, Xerr = calculate_desired_twist(T, Td, Td_next, Kp, Ki, Kd, dt)
	control_speeds = calculate_control_speed(V, Arm_Blist, config, F_Mat, wheels_angles, M0e, Tb0, dt)
	config, wheels_angles = NextState(config, wheels_angles, control_speeds.reshape((9,1)), dt, 2000, F_Mat)
	gripper = state[1]
	if (i % k_per_centi_sec) == 0:
		t += 0.01
		csvrow = np.hstack((config, wheels_angles)).reshape((12,)).tolist()
		csvrow.append(gripper)
		csvobject.writerow(csvrow)
		csvrow = [t] + Xerr.tolist()
		csvXerrObject.writerow(csvrow)

csvfile.close()
csvxerr.close()

# Overshoot Control
config = np.array([0.6, -0.5, 0.5, 0, -0.3, 0.2, -1.6, 0])
print('Generating CSV files for the main task (overshoot)...')
dt = 0.01 / k_per_centi_sec
Kp = 5
Ki = 3
reset_integrator()
# this is to switch the feedward on or off
Kd = 1
t = 0
csvfile = open('../results/overshoot/overshoot.csv', 'w')
csvxerr = open('../results/overshoot/overshoot_xerrlog.csv', 'w')
csvobject = csv.writer(csvfile)
csvXerrObject = csv.writer(csvxerr)
csvobject.writerow(['#phi', 'x', 'y', 'Arm_th1', 'Arm_th2', 'Arm_th3', 'Arm_th4', 'Arm_th5', 'wheel1', 'wheel2', 'wheel3', 'wheel4'])
csvXerrObject.writerow(['t', 'wx', 'wy', 'wz', 'x', 'y', 'z'])
for i in range(len(Traj) - 1):
	state = Traj[i]
	Td = state[0]
	Td_next = Traj[i + 1][0]

	T = Fkin(config, Arm_Blist, M0e, Tb0)
	V, Xerr = calculate_desired_twist(T, Td, Td_next, Kp, Ki, Kd, dt)
	control_speeds = calculate_control_speed(V, Arm_Blist, config, F_Mat, wheels_angles, M0e, Tb0, dt)
	config, wheels_angles = NextState(config, wheels_angles, control_speeds.reshape((9,1)), dt, 2000, F_Mat)
	gripper = state[1]
	if (i % k_per_centi_sec) == 0:
		t += 0.01
		csvrow = np.hstack((config, wheels_angles)).reshape((12,)).tolist()
		csvrow.append(gripper)
		csvobject.writerow(csvrow)
		csvrow = [t] + Xerr.tolist()
		csvXerrObject.writerow(csvrow)

csvfile.close()
csvxerr.close()

# new task
config = np.array([0.6, -0.5, 0.5, 0, -0.3, 0.2, -1.6, 0])
Tse = [[0, 0, 1, 0], [0, 1, 0, 0],[-1, 0, 0, 0.5],[0, 0, 0, 1]]
Tscinitial = [[0.70711,-0.70711,0,0.70711],[0.70711,0.70711,0,0.70711],[0,0,1,0.025],[0,0,0,1]]
Tscgoal = [[-0.70711,-0.70711,0,-0.70711],[0.70711,-0.70711,0,0.70711],[0,0,1,0.025],[0,0,0,1]]
Tcegrasp = [[0, 0, 1, 0], [0, 1, 0, 0],[-1, 0, 0, 0],[0, 0, 0, 1]]
Tcestand = [[0, 0, 1, 0], [0, 1, 0, 0],[-1, 0, 0, 0.4],[0, 0, 0, 1]]
wheels_angles = np.array([0, 0, 0, 0])

print('Generating Trajectory for the new task...')
Traj = TrajectoryGenerator(Tse, Tscinitial, Tscgoal, Tcestand, Tcegrasp, 3, 4, k_per_centi_sec)
print('Generating CSV files for the new task...')

dt = 0.01 / k_per_centi_sec
Kp = 5
Ki = 1
reset_integrator()
# this is to switch the feedward on or off
Kd = 1
t = 0
csvfile = open('../results/newTask/newTask.csv', 'w')
csvxerr = open('../results/newTask/newTask_xerrlog.csv', 'w')
csvobject = csv.writer(csvfile)
csvXerrObject = csv.writer(csvxerr)
csvobject.writerow(['#phi', 'x', 'y', 'Arm_th1', 'Arm_th2', 'Arm_th3', 'Arm_th4', 'Arm_th5', 'wheel1', 'wheel2', 'wheel3', 'wheel4'])
csvXerrObject.writerow(['t', 'wx', 'wy', 'wz', 'x', 'y', 'z'])
for i in range(len(Traj) - 1):
	state = Traj[i]
	Td = state[0]
	Td_next = Traj[i + 1][0]

	T = Fkin(config, Arm_Blist, M0e, Tb0)
	V, Xerr = calculate_desired_twist(T, Td, Td_next, Kp, Ki, Kd, dt)
	control_speeds = calculate_control_speed(V, Arm_Blist, config, F_Mat, wheels_angles, M0e, Tb0, dt)
	config, wheels_angles = NextState(config, wheels_angles, control_speeds.reshape((9,1)), dt, 2000, F_Mat)
	gripper = state[1]
	if (i % k_per_centi_sec) == 0:
		t += 0.01
		csvrow = np.hstack((config, wheels_angles)).reshape((12,)).tolist()
		csvrow.append(gripper)
		csvobject.writerow(csvrow)
		csvrow = [t] + Xerr.tolist()
		csvXerrObject.writerow(csvrow)

csvfile.close()
csvxerr.close()