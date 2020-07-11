function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

m = params.mass;
g = params.gravity;
K_p_z = 80;
K_v_z = 20;
K_p_phi = 1000;
K_v_phi = 10;
K_p_y = 20;
K_v_y = 5;
phi_c = -(des_state.acc(1) + K_v_y * (des_state.vel(1) - state.vel(1)) + K_p_y * (des_state.pos(1) - state.pos(1))) / g;
u1 = m * (g + des_state.acc(2) + K_v_z * (des_state.vel(2) - state.vel(2)) + K_p_z * (des_state.pos(2) - state.pos(2)));
u2 = params.Ixx * (K_p_phi * (phi_c - state.rot) + K_v_phi * (0 - state.omega));
end

