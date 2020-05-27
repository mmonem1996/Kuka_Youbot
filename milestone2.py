import numpy as np 
import modern_robotics as mr

def TrajectoryGenerator(Tse ,Tsci, Tscf, Tcestand, Tcegrap, Tf_screw, Tf_cartesian, k_per_centi_sec):
	Tse = np.array(Tse)
	Tsci = np.array(Tsci)
	Tscf = np.array(Tscf)
	Tcestand = np.array(Tcestand)
	Tcegrap = np.array(Tcegrap)
	gripper_state = 0
	# segment 1: from starting to standoff
	T1 = Tse
	T2 = np.dot(Tsci, Tcestand)
	Traj_seg = mr.ScrewTrajectory(T1, T2, Tf_screw, 100*k_per_centi_sec*Tf_screw, 3)
	Trajectory = [(T, gripper_state) for T in Traj_seg]
	# segment 2: from standoff position (directly bove the block) to the grasping position
	T1 = T2
	T2 = np.dot(Tsci, Tcegrap)
	Traj_seg = mr.ScrewTrajectory(T1, T2, Tf_cartesian, 100*k_per_centi_sec*Tf_cartesian, 3)
	# the operator '+=' will append the return value to Traj_seg list
	Trajectory += [(T, gripper_state) for T in Traj_seg]
	# segment 3: set end-effector to grasp
	gripper_state = 1
	Trajectory += [(T2, gripper_state) for _ in range(100*k_per_centi_sec)]
	# segment 4: return to standoff position
	Ttemp = T1
	T1 = T2
	T2 = Ttemp
	Traj_seg = mr.ScrewTrajectory(T1, T2, Tf_cartesian, 100*k_per_centi_sec*Tf_cartesian, 3)
	Trajectory += [(T, gripper_state) for T in Traj_seg]
	# segment 5: to the final standoff position (dorectly above the target)
	T1 = T2
	T2 = np.dot(Tscf, Tcestand)
	Traj_seg = mr.ScrewTrajectory(T1, T2, Tf_screw, 100*k_per_centi_sec*Tf_screw, 3)
	Trajectory += [(T, gripper_state) for T in Traj_seg]
	# segment 6: from standoff position (directly bove the block) to the grasping position
	T1 = T2
	T2 = np.dot(Tscf, Tcegrap)
	Traj_seg = mr.ScrewTrajectory(T1, T2, Tf_cartesian, 100*k_per_centi_sec*Tf_cartesian, 3)
	Trajectory += [(T, gripper_state) for T in Traj_seg]
	# segment 7: set end-effector to release
	gripper_state = 0
	Trajectory += [(T2, gripper_state) for _ in range(100*k_per_centi_sec)]
	# segment 8: return to standoff position
	Ttemp = T1
	T1 = T2
	T2 = Ttemp
	Traj_seg = mr.ScrewTrajectory(T1, T2, Tf_cartesian, 100*k_per_centi_sec*Tf_cartesian, 3)
	Trajectory += [(T, gripper_state) for T in Traj_seg]

	return Trajectory