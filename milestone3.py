import numpy as np 
import scipy.linalg as sc
import modern_robotics as mr
from milestone1 import NextState

wheels_influence_factor = 8
pinv_cutt_off_threshold = 0.0001
# this variable will be used to accumulate (integrate) error as would an integrator
cdt_integrator = np.zeros((6,))

def reset_integrator():
	global cdt_integrator
	cdt_integrator = np.zeros((6,))

def testJointsLimits(config):
	# config = phi, x, y, th1, th2, th3, th4, th5
	config = config.tolist()
	violation_test_result = [False] * len(config)
	# Arm joint 2
	if (config[4] < -0.75*np.pi) or (config[4] > -0.2):
		violation_test_result[4] = True
	# Arm joint 3
	if config[5] < -1.7:
		violation_test_result[5] = True
	# Arm joint 4
	if config[6] < -1.78:
		violation_test_result[6] = True
	return violation_test_result

def Fkin(config, Baxis_mat, M0e, Tb0):
	# calculate forward kinematics of Kuka youbot
	Baxis_mat = np.array(Baxis_mat)
	config = np.array(config)
	M0e = np.array(M0e)
	Tb0 = np.array(Tb0)
	# Tsb the description of the chassis frame with respect to the spatial frame s
	chassis_theta ,chassis_x, chassis_y = config[0], config[1], config[2]
	cos_th = np.cos(chassis_theta)
	sin_th = np.sin(chassis_theta)
	Tsb = [[cos_th, -sin_th, 0, chassis_x], [sin_th, cos_th, 0, chassis_y], [0, 0, 1, 0.0963],[0, 0, 0, 1]]
	# T0e is the forward kinematics of the arm, which gives ends-effector description wrt the base of the arm frame 0
	arm_joints_value = config[3:8]
	T0e = mr.FKinBody(M0e, Baxis_mat, arm_joints_value)
	Tse = Tsb @ Tb0 @ T0e
	return Tse

def get_jacobian(Baxis_mat, arm_joints_value, F_matrix, M0e, Tb0):
	Baxis_mat = np.array(Baxis_mat)
	arm_joints_value = np.array(arm_joints_value)
	# the F_matrix will be weighted by wheels_influence_factor
	# if the value if this factor is large, the pseudo inverse will output larger wheel speeds
	F_matrix = np.array(F_matrix)*(wheels_influence_factor)
	M0e = np.array(M0e)
	Tb0 = np.array(Tb0)

	T0e = mr.FKinBody(M0e, Baxis_mat, arm_joints_value)
	Teb = mr.TransInv(Tb0 @ T0e)
	# constituting base jacobian or F6(spatial) matrix relating the base wheel speed to the end-effector speed
	_ ,cols = F_matrix.shape
	z_vector = np.zeros((1, cols))
	F_matrix = np.vstack((z_vector, z_vector, F_matrix, z_vector))
	Base_Jacobian = mr.Adjoint(Teb) @ F_matrix
	# constituting the arm jacobian
	Arm_Jacobian = mr.JacobianBody(Baxis_mat, arm_joints_value)

	jacobian = np.hstack((Base_Jacobian, Arm_Jacobian))

	return jacobian

def calculate_control_speed(V, Baxis_mat, config, F_matrix, wheels_angles, M0e, Tb0, dt):
	J = get_jacobian(Baxis_mat, config[3:8], F_matrix, M0e, Tb0)
	control_speeds = sc.pinv2(J, pinv_cutt_off_threshold) @ V
	config, wheels_angles = NextState(config, wheels_angles, control_speeds.reshape((9,1)), dt, 0, F_matrix)
	# test if any joint exceeds limits
	violations = testJointsLimits(config)
	# if any joint is found to be exceeding limit, the associated jacobian column will be zeroed out and the twist will be recalculated
	recalculate_twist = True if any(violations) else False
	for i in range(3,len(violations)):
		if violations[i] == True:
			J[0:6, i + 1] = np.zeros((6,))

	if recalculate_twist:
		control_speeds = sc.pinv2(J, pinv_cutt_off_threshold) @ V
	# this is to balance the effect of multiplying the Chassis jacobian by the factor
	control_speeds[0:4] = control_speeds[0:4]*(wheels_influence_factor)
	return control_speeds

def calculate_desired_twist(Tse, Tsed, Tsd, Kp, Ki, Kd, dt):
	# Tse: is the current actual end_effector frame
	# Tsed: is the current referrance end_effector frame on the trajectory
	# Tsd: is the next referrance end_effector frame on the trajectory
	# Kp, Ki: the proportional and the integrator gain
	# Kd: usually 1 or 0 to switch the feedforward on or off
	Tse = np.array(Tse)
	Tsed = np.array(Tsed)
	Tsd = np.array(Tsd)

	Xerr_se3 = mr.MatrixLog6(mr.TransInv(Tse) @ Tsed)
	Xerr = mr.se3ToVec(Xerr_se3)
	global cdt_integrator
	cdt_integrator += dt * Xerr
	# if kd = 0, then feedforward Vd = 0
	Vd = (1/dt) * mr.se3ToVec(mr.MatrixLog6(mr.TransInv(Tsed) @ Tsd)) if Kd != 0 else np.zeros((6,))
	# feedforward + feedback
	V = (Kd * mr.Adjoint(mr.TransInv(Tse) @ Tsed) @ Vd) + Kp*Xerr + Ki*cdt_integrator
	return V, Xerr