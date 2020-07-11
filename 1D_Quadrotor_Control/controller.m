function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

u = 0;
e = s_des(1) - s(1);
e_dot = s_des(2) - s(2);
K_p = 15000;
K_v = 18;
m = params.mass;
g = params.gravity;
u = m * (g + K_p * e + K_v * e_dot + g);


end

