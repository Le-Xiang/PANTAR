function prosthetic_foot_sim_fixed_step()
    % ----------------------------
    % Parameter initialization (same as previous code)
    % ----------------------------
    spring_len_array = [];
    assignin('base', 'spring_len_array', spring_len_array);
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
    
    % Physical parameters
    I_a = 0.025;        % Ankle joint inertia (kg·m^2)
    I_t = 0.000062327;  % Toe joint inertia (kg·m^2)
    m_t = 0.135;        % Toe mass (kg)
    d = 0.19;           % Distance toe → ankle (m)
    eta = 0.9;          % Efficiency factor
    
    % Spring parameters
    k1_1 = 100440;    % Plantarflexion spring stiffness (N/m)
    k1_2 = 100440;    % Dorsiflexion spring stiffness (N/m)
    k3   = 2380;      % Heel spring stiffness (N/m)
    k2   = 3.225;     % Toe joint spring stiffness (N·m/rad)
    
    % Gear and transmission
    n = 2;             % Ankle gear ratio
    n_t = 0.5;         % Toe gear ratio
    p_m_per_rev = 0.01; % Leadscrew pitch (m/rev)
    p = p_m_per_rev / (2 * pi); % m/rad
    
    % Initial state
    theta_a0 = deg2rad(-5); % Initial ankle angle
    theta_t0 = deg2rad(12); % Initial toe angle
    x_m = 1.6/100;          % Leadscrew initial position (m)
    
    % Damping parameters
    b_a = 1.5;     % Ankle damping (N·m·s/rad)
    M_f = 0.6;     % Friction element (constant opposing velocity)
    b_t = 0.008;   % Toe joint damping (N·m·s/rad)
    b_m = 0;       % Motor damping (N·m·s/rad), assumed 0 due to feedback
    
    % Total inertia
    I_toe_at_ankle = I_t + m_t * d^2;
    I_a_total = I_a + I_toe_at_ankle;
    
    % ----------------------------
    % Fixed-step simulation (1kHz)
    % ----------------------------
    dt = 0.001;       % 1 ms timestep (1 kHz)
    t = 0:dt:1.1;     % Sim time 0–1.1 s
    N = length(t);
   
    % State vector [theta_a, omega_a, x_m, omega_m, theta_t, omega_t, int_e_a, int_e_t]
    y = zeros(N, 8);
    y(1,:) = [theta_a0; 0; x_m; 0; theta_t0; 0; 0; 0]';
    
    % PID parameters
    Kp_a = 300; Ki_a = 0;   Kd_a = 10;    % Ankle
    Kp_t = 90;  Ki_t = 100; Kd_t = 0.09;  % Toe
   
    % ----------------------------
    % Main fixed-step loop (Euler)
    % ----------------------------
    for k = 1:N-1
        t_current = t(k);

        % Controller (1 kHz update)
        [omega_m, tau_pid_t] = controller(t_current, y(k,:), Kp_a, Ki_a, Kd_a, Kp_t, Ki_t, Kd_t, dt);

        % Compute dynamics
        dy = dynamics_v7(t_current, y(k,:), I_a_total, I_t, ...
            k1_1, k1_2, n, p, k3, n_t, b_a, b_t, b_m, eta, k2, ...
            omega_m, tau_pid_t, M_f);

        % Euler integration
        y(k+1,:) = y(k,:) + dy' * dt;
    end
    
    % ----------------------------
    % Post-processing
    % ----------------------------
    spring_len_array = evalin('base', 'spring_len_array');
    fprintf('min spring len = %.4f m\n', min(spring_len_array));
    fprintf('max spring len = %.4f m\n', max(spring_len_array));
    spring_len3_array = evalin('base', 'spring_len3_array');
    fprintf('min spring len3 = %.4f m\n', min(spring_len3_array));
    fprintf('max spring len3 = %.4f m\n', max(spring_len3_array));
    x_m_array = evalin('base', 'x_m_array');
    fprintf('min x m = %.4f m\n', min(x_m_array));
    fprintf('max x m = %.4f m\n', max(x_m_array));

    omega_m_array = evalin('base', 'omega_m_array');
    fprintf('min omega m = %.4f rad/s\n', min(abs(omega_m_array)));
    fprintf('max omega m = %.4f rad/s\n', max(abs(omega_m_array)));
    Q1 = prctile(abs(omega_m_array), 97.5);
    fprintf('Q1 (97.5%%) = %.4f rad/s\n', Q1);

    omega_t_array = evalin('base', 'omega_t_array');
    fprintf('min omega t = %.4f rad/s\n', min(abs(omega_t_array)));
    fprintf('max omega t = %.4f rad/s\n', max(abs(omega_t_array)));
    Q2 = prctile(abs(omega_t_array), 97.5);
    fprintf('Q2 (97.5%%) = %.4f rad/s\n', Q2);

    tau_pid_t_array = evalin('base', 'tau_pid_t_array');
    fprintf('min tau_pid_t = %.4f Nm\n', min(abs(tau_pid_t_array)));
    fprintf('max tau_pid_t = %.4f Nm\n', max(abs(tau_pid_t_array)));
    Q3 = prctile(abs(tau_pid_t_array), 97.5);
    fprintf('Q3 (97.5%%) = %.4f Nm\n', Q3);
    
    % Tracking errors
    theta1_actual = y(:,1);  % Ankle
    theta2_actual = y(:,5);  % Toe
    theta1_desired = arrayfun(@ankle_trajectory_real, t);
    theta2_desired = arrayfun(@toe_trajectory_real, t);
    
    ankle_error = abs(theta1_actual - theta1_desired);
    toe_error   = abs(theta2_actual - theta2_desired);
    
    ankle_error_sum = sum(ankle_error(:));
    toe_error_sum   = sum(toe_error(:));

    fprintf('Cumulative ankle error: %.4f rad\n', ankle_error_sum);
    fprintf('Cumulative toe error: %.4f rad\n', toe_error_sum);

    % Example plots
    h2 = figure();
    plot(t, y(:,1)*180/pi, 'LineWidth', 2); hold on;
    plot(t, arrayfun(@ankle_trajectory_real, t)*180/pi, '--r', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    title('Ankle joint tracking (solid = actual, dashed = desired)');
    legend('Actual angle', 'Reference');
    grid on;
    
    h3 = figure();
    plot(t, y(:,5)*180/pi, 'LineWidth', 2); hold on;
    plot(t, arrayfun(@toe_trajectory_real, t)*180/pi, '--r', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    title('Toe joint tracking (solid = actual, dashed = desired)');
    legend('Actual angle', 'Reference');
    grid on;

    fprintf('Controller calls: %d times\n', length(t));
    fprintf('Estimated control frequency: %.2f Hz\n', length(t) / 1.1);
end

