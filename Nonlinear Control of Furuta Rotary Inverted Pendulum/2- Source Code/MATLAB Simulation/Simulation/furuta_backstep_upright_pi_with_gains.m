function tau = furuta_backstep_upright_pi_with_gains(x,k1,k2,k3)

theta    = x(1);
theta_dot= x(2);
alpha    = x(3);
alpha_dot= x(4);

% Physical parameters
m_p = 0.15;
m_a = 0.05;
L_a = 0.2;
L_p = 0.3;
J_a = 0.02;
J_p = 0.002;
g   = 9.81;

sin_a = sin(alpha);
cos_a = cos(alpha);
La_eff = L_a + (L_p/2)*sin_a;

A = J_a + m_p*La_eff^2 + J_p*sin_a^2 + 1e-6;
B = m_p*L_p*La_eff*cos_a + 2*J_p*sin_a*cos_a;
C = J_p + (m_p*L_p^2)/4 + 1e-6;

z1 = alpha;
z2 = alpha_dot + k1*z1 + 0.1*theta;
z3 = theta_dot + z1 + (k1 + k2)*z2;

tau = B*alpha_dot*theta_dot + ...
      A*(-k3*z3 - z2 - (k1+k2)*((0.5*B*theta_dot^2 + m_p*g*(L_p/2)*sin_a)/C + k1*alpha_dot));

tau = max(min(tau, 1), -1);
end
