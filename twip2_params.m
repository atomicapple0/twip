function [whdd thdd] = twip2_params( wh, whd, th, thd, motor_command )
% compute a two wheeled inverted pendulum's forward dynamics

% This version uses the wheel angle relative to the body as a state variable
% (which we measure directly), rather than the wheel angle relative to
% vertical (which is not measured directly).

% Note that the arguments now include the wheel state

% wh and th are in radians
% th and thd are in radians/second
% command is in motor_command units

% The parameters we assume we know perfectly
r_w = 0.0323; % m - wheel radius - easy to measure
g = 9.81; % m/s^2

% My best guess as to parameter values for unknown parameters
m_w = 0.173; % kg
m_p = 0.826; % kg
I_w = 0.0066; % kg-m^2
l_p = 0.043; % m
I_p = 0.00084; % kg-m^2
motor_viscosity = 0.0095; % Nm-s/radian
torque_scale_factor = 0.004; % Nm/command_units

% This version is expressed in terms of 3 unknown inertial parameters p1, p2, and p3
p1 = I_w + (m_p + m_w)*r_w*r_w; % a moment of inertia about the contact point
p2 = m_p*l_p; % a mass moment for the pendulum
p3 = I_p + m_p*l_p*l_p; % a moment of inertia of the pendulum about the wheel axle

% Compute the "mass" matrix in terms of the three inertial parameters p_i:
m12 = p1 + p2*r_w*cos(th);
M = [ p1  m12
      m12 (p1 + 2*p2*r_w*cos(th) + p3) ];
Minv = inv(M);

% Compute the right hand side (rhs) of the dynamics equations
v_exp = p2*r_w*thd*thd*sin(th);
v = [ (-torque_scale_factor*motor_command + v_exp - motor_viscosity*whd) (v_exp + p2*g*sin(th)) ];
rhs = transpose( v );

% And compute the accelerations
result = Minv*rhs;
whdd = result(1);
thdd = result(2);

end
