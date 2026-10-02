function [dx1,dx2, dx3, dx4] = furuta_f(x, tau)

% --- Physical parameters (edit to match your hardware) ---
m_p = 0.15;     % pendulum mass (kg)
m_a = 0.05;     % rotary arm mass (kg)    (if not used set reasonable)
L_a = 0.2;    % distance from vertical axis to pendulum pivot (m)
L_p = 0.3;    % pendulum length (m)
J_a = 0.02;    % rotary arm inertia (kg*m^2)
J_p = 0.002;   % pendulum inertia (kg*m^2)
g  = 9.81;     % gravity (m/s^2)

% State unpack
theta    = x(1);
theta_dot= x(2);
alpha    = x(3);
alpha_dot= x(4);

% Parameters read as block parameters (declare them in Edit Data as Parameter)
% Example: J_a = 0.02; J_p = 0.002; m_p = 0.15; L_a = 0.2; L_p = 0.3; g = 9.81;
% (Do NOT redefine them here; set them in base workspace/model workspace.)
% They must be visible to Simulink and declared in the MATLAB Function block as Parameter.

% Precompute
sin_a = sin(alpha);
cos_a = cos(alpha);
La_eff = L_a + (L_p/2)*sin_a;

A = J_a + m_p*La_eff^2 + J_p*sin_a^2 + 1e-6;                % denom for theta_ddot
B = m_p*L_p*La_eff*cos_a + 2*J_p*sin_a*cos_a;        % coupling term
C = J_p + (m_p*L_p^2)/4 + 1e-6;                             % denom for alpha_ddot

% State derivatives
dx1 = theta_dot;
dx2 = ( tau - B * alpha_dot * theta_dot ) / A;     % theta_ddot
dx3 = alpha_dot;
dx4 = ( 0.5*B * theta_dot^2 + m_p*g*(L_p/2)*sin_a ) / C;  % alpha_ddot

end