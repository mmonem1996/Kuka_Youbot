# Kuka_Youbot

This code simulate the control of Kuka YouBot acheiving a pick and place task. the input of the program is:
- the initial configuration of the robot (config)
- the initial reference configuration of the robot (Tse)
- the initial position and orientation of the block (Tscinitial)
- the Goal position and orientation of the block (Tscgoal)
- the stand off frame description with respect to the block frame (Tcestand)
- the grasp frame description with respect to the block frame (Tcegrasp)

The program will output csv files for the animation of the robot in CoppeliaSim as well as a csv showing the convergence of the error twist (Xerr) to zero with time.

You will find the code divided by milestones, and the main code is : code.py, to facilitate reading
