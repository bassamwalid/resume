function best = tune_backstep_gains()

    % -------------------------
    % PARAMETERS TO SWEEP
    % -------------------------
    k1_vals = 1:2:100;
    k2_vals = 1:2:100;
    k3_vals = 0:1:50;

    % -------------------------
    % SIMULATION PARAMETERS
    % -------------------------
    dt = 0.002;          % integration step
    T  = 20;             % simulate 10 seconds
    t  = 0:dt:T;
    N  = length(t);

    alpha_tol = 0.1;     % |alpha| < 0.1 rad
    stable_time = 5;     % must remain stable for 5 continuous seconds
    stable_samples = stable_time / dt;

    % -------------------------
    % INITIAL CONDITION
    % -------------------------
    x0 = [0; 0; 0.2; 0];    % your IC

    % -------------------------
    % BEST RESULT TRACKER
    % -------------------------
    best.found = false;
    best.k1 = NaN; best.k2 = NaN; best.k3 = NaN;

    fprintf("\nStarting search...\n")

    % -------------------------
    % SWEEP k1 k2 k3
    % -------------------------
    for k1 = k1_vals
        for k2 = k2_vals
            for k3 = k3_vals

                fprintf("Testing k1=%g  k2=%g  k3=%g ...\n",k1,k2,k3);

                % Simulate full dynamics
                x = x0;
                alpha_history = zeros(1,N);

                for i = 1:N
                    tau = furuta_backstep_upright_pi_with_gains(x,k1,k2,k3);
                    dx  = furuta_f(x, tau);
                    x   = x + dx * dt;
                    alpha_history(i) = x(3);
                end

                % Check stability window
                window_ok = abs(alpha_history) < alpha_tol;
                conv = find_consecutive(window_ok, stable_samples);

                if conv
                    fprintf(">>> Found stable gains: k1=%g k2=%g k3=%g\n",k1,k2,k3);
                    best.found = true;
                    best.k1=k1; best.k2=k2; best.k3=k3;
                    return;
                end

            end
        end
    end

    fprintf("No stable gains found in given ranges.\n");

end
