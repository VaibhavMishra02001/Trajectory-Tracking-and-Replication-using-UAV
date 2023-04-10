function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

% Thrust

% Moment
%CONTROLLER  Controller for the quadrotor


pos = state.pos;
vel = state.vel;
rot = state.rot;
omega = state.omega;

pos_des = des_state.pos;
vel_des = des_state.vel;
acc_des = des_state.acc;
yaw_des = des_state.yaw;


m = params.mass;
g = params.gravity;
L = params.arm_length;


Kp_pos = [25; 25; 300]; 
Kd_pos = [10; 10; 100]; 
Kp_phi = 50; 
Kd_phi = 10; 
Kp_theta = 50; 
Kd_theta = 10; 
Kp_psi = 50;
Kd_psi = 10; 

pos_err = pos_des - pos;
vel_err = vel_des - vel;


acc_des(3) = acc_des(3) + g; 
acc_cmd = acc_des + Kp_pos .* pos_err + Kd_pos .* vel_err;


F = m * norm(acc_cmd); 


phi_cmd = (1/g) * (acc_cmd(1)*sin(yaw_des) - acc_cmd(2)*cos(yaw_des));
theta_cmd = (1/g) * (acc_cmd(1)*cos(yaw_des) + acc_cmd(2)*sin(yaw_des));

M = [Kp_phi*(phi_cmd - rot(1)) + Kd_phi*(0 - omega(1));
     Kp_theta*(theta_cmd - rot(2)) + Kd_theta*(0 - omega(2));
     Kp_psi*(yaw_des - rot(3)) + Kd_psi*(0 - omega(3))] * L;



% =================== Your code ends here ===================

end
