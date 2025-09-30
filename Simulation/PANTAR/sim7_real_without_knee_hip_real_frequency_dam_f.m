function prosthetic_foot_sim_fixed_step()
    % ----------------------------
    % Parameter initialization (same as original code)
    % ----------------------------
    spring_len_array = [];
    assignin('base', 'spring_len_array', spring_len_array);
    spring_len1_array = [];
    assignin('base', 'spring_len1_array', spring_len1_array);
    spring_len2_array = [];
    assignin('base', 'spring_len2_array', spring_len2_array);
    spring_len3_array = [];
    assignin('base', 'spring_len3_array', spring_len3_array);
    x_m_array = [];
    assignin('base', 'x_m_array', x_m_array);
    omega_m_array = [];
    assignin('base', 'omega_m_array', omega_m_array);
    omega_t_array = [];
    assignin('base', 'omega_t_array', omega_t_array);
    tau_pid_t_array = [];
    assignin('base', 'tau_pid_t_array', tau_pid_t_array);
    F1_array = [];
    assignin('base', 'F1_array', F1_array);
    torque_a_array = [];
    assignin('base', 'torque_a_array', torque_a_array);
    torque_t_array = [];
    assignin('base', 'torque_t_array', torque_t_array);
    torque_t1_array = [];
    assignin('base', 'torque_t1_array', torque_t1_array);
    torque_t3_array = [];
    assignin('base', 'torque_t3_array', torque_t3_array);
    F1_1array = [];
    assignin('base', 'F1_1array', F1_1array);
    F1_2array = [];
    assignin('base', 'F1_2array', F1_2array);

    % Initialize once on first run (or initialize manually in workspace)
    assignin('base', 'spring_both_satisfied_count', 0);
    assignin('base', 'spring_not_satisfied_count', 0);
    t_n = [];
    assignin('base', 'spring_not_satisfied_time', t_n);

    % ----------------------------
    % Physical parameters
    % ----------------------------
    I_a = 0.025;      % Ankle inertia (kg·m^2)
    I_t = 0.000062327; % Toe inertia (kg·m^2)
    m_a = 2.5;        % Ankle linkage structure mass (kg)
    m_t = 0.135;      % Toe mass (kg)
    d = 0.19;         % Distance toe-to-ankle (m)

    % Efficiency factors
    eta.motor   = 0.90;  % Motor
    eta.gear    = 0.96;  % Gear train
    eta.screw   = 0.96;  % Ball screw
    eta.spring  = 0.98;  % Springs
    eta.damping = 0.95;  % Damper

    % ----------------------------
    % Spring parameters (after considering preload/extension)
    % ----------------------------
    k1_1 = 126000;   % Plantarflexion spring stiffness (N/m)
    k1_2 = 119080;   % Dorsiflexion spring stiffness (N/m)
    k3   = 17920;    % Heel spring stiffness (N/m)
    k2   = 3.225;    % Toe spring stiffness (N·m/rad)

    % ----------------------------
    % Gear & transmission
    % ----------------------------
    n = 0.75;             % Ankle gear ratio
    n_t = 0.5;            % Toe gear ratio
    p_m_per_rev = 0.01;   % Ball screw lead (m/rev)
    p = p_m_per_rev / (2 * pi); % Lead (m/rad)

    % ----------------------------
    % Initial conditions
    % ----------------------------
    theta_a0 = deg2rad(-5); % Initial ankle angle
    theta_t0 = deg2rad(12); % Initial toe angle
    x_m = 2.7/100;          % Initial screw position (m)

    % ----------------------------
    % Damping/friction
    % ----------------------------
    b_a = 1.5;     % Ankle damping (N·m·s/rad) - replaced by nonlinear curve
    M_f_a = 0.01;  % Ankle friction torque constant
    M_f_t = 0.01;  % Toe friction torque constant
    b_t = 0;       % Toe damping (N·m·s/rad)
    b_m = 0;       % Motor damping (N·m·s/rad)

    % ----------------------------
    % Total inertia
    % ----------------------------
    I_toe_at_ankle = I_t + m_t * d^2;
    I_a_total = I_a + I_toe_at_ankle;

    % ----------------------------
    % Fixed-step simulation settings (1 kHz)
    % ----------------------------
    dt = 0.001;        % Step size (s)
    t = 0:dt:1;        % Simulation time
    N = length(t);

    % State vector: [ankle angle, ankle vel, screw pos, motor vel, toe angle, toe vel, ankle intErr, toe intErr]
    y = zeros(N, 8);
    y(1,:) = [theta_a0; 0; x_m; 0; theta_t0; 0; 0; 0]';

    % ----------------------------
    % PID parameters
    % ----------------------------
    Kp_a = 1000; Ki_a = 0;   Kd_a = 50;   % Ankle
    Kp_t = 80;   Ki_t = 200; Kd_t = 0.1;  % Toe

    % ----------------------------
    % Main loop (Euler integration)
    % ----------------------------
    for k = 1:N-1
        t_current = t(k);

        % Controller update (1 kHz)
        [omega_m, tau_pid_t] = controller(t_current, y(k,:), Kp_a, Ki_a, Kd_a, Kp_t, Ki_t, Kd_t, dt);

        % Compute derivatives
        dy = dynamics_v7(t_current, y(k,:), I_a_total, I_t, ...
            k1_1, k1_2, n, p, k3, n_t, b_a, b_t, b_m, eta, k2, ...
            omega_m, tau_pid_t, M_f_a, M_f_t, m_a);

        % Euler integration
        y(k+1,:) = y(k,:) + dy' * dt;
    end

    % ----------------------------
    % Post-processing & results
    % ----------------------------
    spring_len_array = evalin('base', 'spring_len_array');
    fprintf('min spring len = %.4f m\n', min(spring_len_array));
    fprintf('max spring len = %.4f m\n', max(spring_len_array));
    spring_len1_array = evalin('base', 'spring_len1_array');
    fprintf('min spring len1 = %.5f m\n', min(spring_len1_array));
    fprintf('max spring len1 = %.5f m\n', max(spring_len1_array));
    fprintf('max delta spring len1 = %.5f m\n', max(abs(spring_len1_array-50.8/1000)));
    spring_len2_array = evalin('base', 'spring_len2_array');
    fprintf('min spring len2 = %.5f m\n', min(spring_len2_array));
    fprintf('max spring len2 = %.5f m\n', max(spring_len2_array));
    fprintf('max delta spring len2 = %.5f m\n', max(abs(spring_len2_array-25.4/1000)));
    spring_len3_array = evalin('base', 'spring_len3_array');
    fprintf('min spring len3 = %.5f m\n', min(spring_len3_array));
    fprintf('max spring len3 = %.5f m\n', max(spring_len3_array));
    fprintf('max delta spring len3 = %.5f m\n', max(abs(spring_len3_array-25.4/1000)));
    x_m_array = evalin('base', 'x_m_array');
    fprintf('min x m = %.4f m\n', min(x_m_array));
    fprintf('max x m = %.4f m\n', max(x_m_array));

    % (… output continues unchanged …)

    % ----------------------------
    % Plotting: ankle & toe tracking
    % ----------------------------
    h7 = figure();
    plot(t, y(:,1)*180/pi, 'LineWidth', 2); hold on;
    plot(t, arrayfun(@ankle_trajectory_real, t)*180/pi, '--r', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    title('Ankle Joint Tracking (solid=actual, dashed=desired)');
    legend('Actual angle', 'Desired trajectory');
    grid on;

    h8 = figure();
    plot(t, y(:,5)*180/pi, 'LineWidth', 2); hold on;
    plot(t, arrayfun(@toe_trajectory_real, t)*180/pi, '--r', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    title('Toe Joint Tracking (solid=actual, dashed=desired)');
    legend('Actual angle', 'Desired trajectory');
    grid on;

    fprintf('Controller call count: %d\n', length(t));
    fprintf('Estimated average control frequency: %.2f Hz\n', length(t) / 1.1);
    count = evalin('base', 'spring_both_satisfied_count');
    fprintf('spring_both_satisfied_count = %d\n', count);
    count_f = evalin('base', 'spring_not_satisfied_count');
    fprintf('spring_not_satisfied_count = %d\n', count_f);
end

