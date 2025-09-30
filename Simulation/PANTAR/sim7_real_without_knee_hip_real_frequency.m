function prosthetic_foot_sim_v7()
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
    
    % Parameter definitions
    % The part above the ankle joint is < 1 kg, the toe is ~0.1 kg, so total ~3 kg
    I_a = 0.025;      % Footplate inertia (kg·m^2), ~2 kg, with rotating ankle part + extra ~2.5–3.0
    I_t = 0.000062327; % Toe inertia (kg·m^2)
    m_t = 0.135;       % Toe mass (kg)
    d   = 0.19;        % Toe → ankle distance (m)
    eta = 0.9;         % Efficiency factor
    
    % Note: footplate also has motor, etc. inertia not counted here.
    I_toe_at_ankle = I_t + m_t * d^2;
    I_a_total = I_a + I_toe_at_ankle;
    
    % System 1 – ankle joint spring system
    k1_1 = 280200; % Plantarflexion spring stiffness (N/m), left spring
    k1_2 =  70050; % Dorsiflexion spring stiffness (N/m), right spring
    n = 2;         % Gear ratio
    p_m_per_rev = 0.01; % Leadscrew pitch (m/rev) → here set to 10 mm/rev
    p = p_m_per_rev / (2 * pi);  % m/rad
    theta_a0 = deg2rad(0);       % Initial ankle angle
    
    % System 3 – ankle joint spring
    k3 = 2380; % Heel spring stiffness
    
    % System 2 – toe joint
    n_t = 0.5;              % Gear ratio
    theta_t0 = deg2rad(12); % Initial toe angle
    I_actual = 0;           % Placeholder: measured motor current
    k2 = 8.6;               % Toe spring stiffness (N·m/rad)
    
    % Damping
    b_a = 0.05;  % Ankle damping (N·m·s/rad)
    b_t = 0.01;  % Toe damping (N·m·s/rad)
    b_m = 0.005; % Motor shaft damping (toe) (N·m·s/rad)

    % Initial x_m depending on ankle angle
    x_m = 2.075/100; % for 0°
    % x_m = 1.75/100; % for -5°

    % Initial state: [ankle_angle; ankle_vel; screw_pos; motor_vel; toe_angle; toe_vel; int_err_a; int_err_t]
    y0 = [theta_a0; 0; x_m; 0; theta_t0; 0; 0; 0]; 

    tspan = [0 1.1];
    
    [t, y] = ode45(@(t,y) dynamics_v7(t, y, ...
        I_a_total, I_t, k1_1, k1_2, n, p, k3, ...
        n_t, b_a, b_t, b_m, eta, I_actual, k2), tspan, y0);

    % Post-processing
    spring_len_array = evalin('base', 'spring_len_array');
    fprintf('min spring len = %.4f m\n', min(spring_len_array));
    fprintf('max spring len = %.4f m\n', max(spring_len_array));
    spring_len3_array = evalin('base', 'spring_len3_array');
    fprintf('min spring len3 = %.4f m\n', min(spring_len3_array));
    fprintf('max spring len3 = %.4f m\n', max(spring_len3_array));
    x_m_array = evalin('base', 'x_m_array');
    fprintf('min x_m = %.4f m\n', min(x_m_array));
    fprintf('max x_m = %.4f m\n', max(x_m_array));

    omega_m_array = evalin('base', 'omega_m_array');
    fprintf('min omega_m = %.4f rad/s\n', min(abs(omega_m_array)));
    fprintf('max omega_m = %.4f rad/s\n', max(abs(omega_m_array)));
    Q1 = prctile(abs(omega_m_array), 97.5);
    fprintf('Q1 (97.5%%) = %.4f rad/s\n', Q1);

    omega_t_array = evalin('base', 'omega_t_array');
    fprintf('min omega_t = %.4f rad/s\n', min(abs(omega_t_array)));
    fprintf('max omega_t = %.4f rad/s\n', max(abs(omega_t_array)));
    Q2 = prctile(abs(omega_t_array), 97.5);
    fprintf('Q2 (97.5%%) = %.4f rad/s\n', Q2);

    tau_pid_t_array = evalin('base', 'tau_pid_t_array');
    fprintf('min tau_pid_t = %.4f Nm\n', min(abs(tau_pid_t_array)));
    fprintf('max tau_pid_t = %.4f Nm\n', max(abs(tau_pid_t_array)));
    Q3 = prctile(abs(tau_pid_t_array), 97.5);
    fprintf('Q3 (97.5%%) = %.4f Nm\n', Q3);

    % Torque from spring system
    omega_m_array = evalin('base', 'omega_m_array');
    F1_array = evalin('base', 'F1_array');
    F1_array = F1_array*0.01/(2*pi*0.9);

    fprintf('min T1 = %.4f Nm\n', min(abs(F1_array)));
    fprintf('max T1 = %.4f Nm\n', max(abs(F1_array)));

    % Time axis for plotting
    t_total = 1.1;
    N = length(omega_m_array);
    T = linspace(0, t_total, N);
    
    % === Plot omega_m and T1 ===
    h0 = figure;
    subplot(2,1,1);
    plot(T, omega_m_array, 'b', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('\omega_m (rad/s)');
    title('Motor speed \omega_m');
    grid on;
    
    subplot(2,1,2);
    plot(T, F1_array, 'r', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('T_1 (Nm)');
    title('Spring torque T_1');
    grid on;

    % === Plot omega_t ===
    h1 = figure;
    M = length(omega_t_array);
    TT = linspace(0, t_total, M);
    plot(TT, omega_t_array, 'r', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('\omega_t (rad/s)');
    title('Toe joint angular velocity \omega_t');
    grid on;

    % === Tracking results ===
    theta1_actual = y(:,1);  % Ankle
    theta2_actual = y(:,5);  % Toe
    theta1_desired = arrayfun(@ankle_trajectory_real, t);
    theta2_desired = arrayfun(@toe_trajectory_real, t);

    ankle_error = abs(theta1_actual - theta1_desired);
    toe_error   = abs(theta2_actual - theta2_desired);

    fprintf('Cumulative ankle error: %.4f rad\n', sum(ankle_error));
    fprintf('Cumulative toe error: %.4f rad\n', sum(toe_error));

    % === Ankle joint tracking ===
    h2 = figure();
    plot(t, y(:,1)*180/pi, 'LineWidth', 2); hold on;
    plot(t, arrayfun(@ankle_trajectory_step, t)*180/pi, '--r', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    title('Ankle joint tracking (solid=actual, dashed=reference)');
    legend('Actual angle', 'Reference');
    grid on;
    
    % === Toe joint tracking ===
    h3 = figure();
    plot(t, y(:,5)*180/pi, 'LineWidth', 2); hold on;
    plot(t, arrayfun(@toe_trajectory_real, t)*180/pi, '--r', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    title('Toe joint tracking (solid=actual, dashed=reference)');
    legend('Actual angle', 'Reference');
    grid on;

    % === Errors ===
    h4 = figure();
    plot(t, ankle_error, 'r', 'LineWidth', 2); hold on;
    plot(t, toe_error, 'b', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Error (rad)');
    legend('Ankle error', 'Toe error');
    title('Ankle & toe tracking error over time');
    grid on;

    fprintf('Controller calls: %d\n', length(t));
    fprintf('Estimated control frequency: %.2f Hz\n', length(t)/1.1);
end


