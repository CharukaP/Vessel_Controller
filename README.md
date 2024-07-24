## Controller algorithm for vessel control

The model vessel used here is included in 'RK4.m' file which uses Mechanistic modeling and Range Kutta 4 algorithm to prdict next state

The linearization of nonlinear model can be replicated by 'Linearization.m' file

Development of three PID models and tunig them is accessible in 'MIMP_PID_Tune.m' file

PID controller performance can be recreated using 'PID_Controller.m' file

MPC controller can be accessed via mpcDesigner toolbox session in 'MPC_Simulation_final.mat' file. here the linearized plant is used as  the model

NMPC controller performance can be accessed using 'NMPC.m' file

Four corner Dynamic Positioning test results can be recreated using 'NMPC_4DP_test.m' file
