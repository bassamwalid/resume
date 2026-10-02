% =============================================
% AUTO-TUNING SCRIPT FOR FURUTA UPRIGHT STABILIZATION
% Finds k1, k2, k3 such that alpha → 0 in < 10 seconds
% Works with your existing furuta_f.m (gravity term is +sin(alpha))
% =============================================

clear; clc; close all;

% Initial condition (you can change this)
% Try from hanging position (alpha = pi) or near upright
x0 = [0; 0; pi-0.2; 0];    % arm at 0, pendulum almost down but slightly up

tspan = [0 10];            % simulate 10 seconds
options = odeset('RelTol',1e-6,'AbsTol',1e-8);

% Gain search ranges (these are known to contain good solutions)
k1_range = 20:5:80;
k2_range = 5:3:40;
k3_range = 1:1:20;

best_error = inf;
best_gains = [];
best_t = [];
best_x = [];

fprintf('Searching for stabilizing gains... (k1,k2,k3)\n');
fprintf('%3s %5s %5s   %8s   %8s\n','k1','k2','k3','Final|α|','Success?\n');
fprintf('%s\n',repmat('-',1,50));

for k1 = k1_range
    for k2 = k2_range
        for k3 = k3_range
            
            % === Fixed-point controller with current gains ===
            % We define it inside the loop so k1,k2,k3 are captured
            function tau_out = controller(x)
                theta = x(1); theta_dot = x(2);
                alpha = x(3); alpha_dot = x(4);
                
                m_p = 0.15; L_a = 0.2; L_p = 0.3; g = 9.81;
                J_a = 0.02; J_p = 0.002;
                
                sin_a = sin(alpha); cos_a = cos(alpha);
                La_eff = L_a + (L_p/2)*sin_a;
                A = J_a + m_p*La_eff^2 + J_p*sin_a^2 + 1e-6;
                B = m_p*L_p*La_eff*cos_a + 2*J_p*sin_a*cos_a;
                C = J_p + (m_p*L_p^2)/4 + 1e-6;
                
                % Your structure, but with CORRECT gravity sign (+)
                z1 = alpha;                                           % want → 0
                z2 = alpha_dot + k1*z1 + 0.1*theta;                   % your extra term
                z3 = theta_dot + z1 + (k1 + k2)*z2;
                
                alpha_ddot_nat = (0.5*B*theta_dot^2 + m_p*g*(L_p/2)*sin_a) / C;  % CORRECT +
                
                tau_out = B * alpha_dot * theta_dot + ...
                          A * (-k3*z3 - z2 - (k1 + k2)*(alpha_ddot_nat + k1*alpha_dot));
                
                tau_out = max(min(tau_out, 1.5), -1.5);  % reasonable torque limit
            end
            
            % Simulate
            try
                [t, x] = ode45(@(t,x) furuta_f(x, controller(x)), tspan, x0, options);
                
                alpha_final = x(end,3);
                alpha_error = min( abs(alpha_final - 0), abs(alpha_final - 2*pi - 0) ); % robust
                settled = all( abs(x(t>5,3)) < 0.15 );  % stays within ±8.6 deg after 5s
                
                if alpha_error < 0.2 && settled && t(end) >= 9.5
                    fprintf('\x001b[32m%3d %5d %5d   %8.4f   \x001b[32mSUCCESS!\x001b[0m\n', k1,k2,k3,alpha_error);
                    if alpha_error < best_error
                        best_error = alpha_error;
                        best_gains = [k1 k2 k3];
                        best_t = t;
                        best_x = x;
                    end
                else
                    fprintf('%3d %5d %5d   %8.4f   failed\n', k1,k2,k3,alpha_error);
                end
            catch
                fprintf('%3d %5d %5d   crash\n', k1,k2,k3);
            end
        end
    end
end

% === Final Result ===
if ~isempty(best_gains)
    fprintf('\n\x001b[1;32mBEST FOUND GAINS: k1 = %.1f, k2 = %.1f, k3 = %.1f\x001b[0m\n', best_gains);
    fprintf('Final alpha = %.4f rad (error = %.4f)\n', best_x(end,3), best_error);
    
    % Plot result
    figure('Position',[100 100 1000 600]);
    subplot(3,1,1); plot(best_t, best_x(:,1)); ylabel('\theta (rad)'); title('Best Stabilization Result');
    subplot(3,1,2); plot(best_t, best_x(:,3)); ylabel('\alpha (rad)'); hold on; yline(0,'r--'); yline(pi,'k:');
    subplot(3,1,3); tau_hist = arrayfun(@(i) controller(best_x(i,:)), 1:size(best_x,1)); 
                plot(best_t, tau_hist); ylabel('Torque (Nm)'); xlabel('Time (s)');
else
    fprintf('No stabilizing gains found in the range.\n');
end