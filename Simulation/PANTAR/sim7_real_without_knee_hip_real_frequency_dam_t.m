function prosthetic_foot_sim_fixed_step()
    % ----------------------------
    % Initialization (same as your original code)
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
 
    % Only initialize on first run (or initialize once in the base workspace)
    % if evalin('base', 'exist(''spring_both_satisfied_count'', ''var'') == 0')
    %     assignin('base', 'spring_both_satisfied_count', 0);
    % end
    %
    % if evalin('base', 'exist(''spring_not_satisfied_count'', ''var'') == 0')
    %     assignin('base', 'spring_not_satisfied_count', 0);
    % end

    assignin('base', 'spring_both_satisfied_count', 0);
    assignin('base', 'spring_not_satisfied_count', 0);
    t_n = [];
    assignin('base', 'spring_not_satisfied_time', t_n);
 
    % Physical parameters
    I_a = 0.025;        % Ankle joint moment of inertia (kg·m^2) 0.025
    I_t = 0.000062327;  % Toe joint moment of inertia (kg·m^2)
    m_a = 2.5;          % Mass of the ankle-linked structure
    m_t = 0.135;        % Toe mass (kg)
    d = 0.19;           % Distance from toe joint to ankle joint (m)
    % eta = 0.9;        % Efficiency factor
    eta.motor  = 0.90;  % Motor efficiency
    eta.gear   = 0.96;  % Gear train
    eta.screw  = 0.96;  % Ball screw
    eta.spring = 0.98;  % Springs
    eta.damping= 0.95;  % Dampers
    
    % % Spring parameters (older set)
    % k1_1 = 280200;    % Plantarflexion spring stiffness (N/m)
    % k1_2 = 282160;    % Dorsiflexion spring stiffness (N/m)
    % k3 = 2040;        % Heel spring stiffness (N/m)
    % k2 = 3.225;       % Toe joint spring stiffness (N·m/rad)

    % Spring parameters (considering extension/compression limits)
    k1_1 = 126000;   % Plantarflexion spring stiffness (N/m), die spring*2
    k1_2 = 119080;   % Dorsiflexion spring stiffness (N/m), compression spring*4
    k3   = 17920;    % Heel spring stiffness (N/m), tension spring*4
    k2   = 3.225;    % Toe joint spring stiffness (N·m/rad), torsion spring*3
    
    % Gearing & transmission
    n  = 0.75;            % Ankle gear ratio
    n_t = 0.5;            % Toe gear ratio
    p_m_per_rev = 0.01;   % Screw lead (m/rev)
    p = p_m_per_rev / (2 * pi); % m/rad
    
    % Initial state
    theta_a0 = deg2rad(-5); % Initial ankle angle
    theta_t0 = deg2rad(12); % Initial toe angle
    x_m = 2.7/100;          % Initial screw position (m)
    
    % Damping parameters
    b_a  = 1.5;    % Ankle damping (N·m·s/rad) — replaced by b_a curve below
    M_f_a = 0.01;  % Coulomb-like friction at ankle (constant opposing velocity)
    M_f_t = 0.01;  % Coulomb-like friction at toe (small inherent friction)
    b_t  = 0;      % Toe viscous damping (N·m·s/rad)
    b_m  = 0;      % Motor viscous damping (N·m·s/rad) — RoboMaster with feedback → ~0
    
    % Total inertia about the ankle
    I_toe_at_ankle = I_t + m_t * d^2;
    I_a_total = I_a + I_toe_at_ankle;
    
    % ----------------------------
    % Fixed-step simulation setup (1 kHz)
    % ----------------------------
    dt = 0.001;      % 1 ms time step (1 kHz)
    t = 0:dt:1;      % Simulation time 0–1 s
    N = length(t);
   
    % State vector: [theta_a, omega_a, x_m, omega_m, theta_t, omega_t, integral_e_a, integral_e_t]
    y = zeros(N, 8);
    y(1,:) = [theta_a0; 0; x_m; 0; theta_t0; 0; 0; 0]';
    
    % PID gains
    Kp_a = 1000; Ki_a = 0;   Kd_a = 50;   % Ankle
    Kp_t = 80;   Ki_t = 200; Kd_t = 0.1;  % Toe
   
    % ----------------------------
    % Fixed-step main simulation loop
    % ----------------------------
    % Euler integration
    for k = 1:N-1
        t_current = t(k);

        % Controller (updated at 1 kHz)
        [omega_m, tau_pid_t] = controller(t_current, y(k,:), ...
            Kp_a, Ki_a, Kd_a, Kp_t, Ki_t, Kd_t, dt);

        % Dynamics derivative
        dy = dynamics_v7(t_current, y(k,:), I_a_total, I_t, ...
            k1_1, k1_2, n, p, k3, n_t, b_a, b_t, b_m, eta, k2, ...
            omega_m, tau_pid_t, M_f_a, M_f_t, m_a);

        % Euler state update
        y(k+1,:) = y(k,:) + dy' * dt;
    end
    
    % ----------------------------
    % Post-processing & analysis
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
     
    % Visualization
    h1 = figure();
    visualize_motion_v7(t, y);
    
    % Errors
    theta1_actual = y(:,1);  % Ankle
    theta2_actual = y(:,5);  % Toe
    
    % Desired trajectories
    theta1_desired = arrayfun(@ankle_trajectory_real, t);
    theta2_desired = arrayfun(@toe_trajectory_real, t);
    
    % Absolute errors (deg)
    ankle_error =  rad2deg(abs(theta1_actual - theta1_desired));
    toe_error   =  rad2deg(abs(theta2_actual - theta2_desired));
    
    % Accumulated error (scalar)
    ankle_error_sum = sum(ankle_error(:));
    toe_error_sum   = sum(toe_error(:));

    % Print error results
    fprintf('Accumulated ankle error: %.4f deg\n', ankle_error_sum);
    fprintf('Accumulated toe error: %.4f deg\n',   toe_error_sum);

    % From base workspace
    omega_m_array = evalin('base', 'omega_m_array');
    F1_array = evalin('base', 'F1_array');
    fprintf('min F1 = %.4f N', min(abs(F1_array)));
    fprintf('max F1 = %.4f N\n', max(abs(F1_array)));

    F1_1array = evalin('base', 'F1_1array');
    fprintf('min F1_1 = %.4f N', min(abs(F1_1array)));
    fprintf('max F1_1 = %.4f N\n', max(abs(F1_1array)));
   
    F1_2array = evalin('base', 'F1_2array');
    fprintf('min F1_2 = %.4f N', min(abs(F1_2array)));
    fprintf('max F1_2 = %.4f N\n', max(abs(F1_2array)));

    F1_array = F1_array*0.01/(2*pi*eta.screw);

    fprintf('min T1 = %.4f Nm', min(abs(F1_array)));
    fprintf('max T1 = %.4f Nm\n', max(abs(F1_array)));
    Q4 = prctile(abs(F1_array ), 97.5);
    fprintf('Q3 (97.5%%) = %.4f Nm\n', Q4);
    
    t_n = evalin('base', 'spring_not_satisfied_time');
    fprintf('t_n = %.4f s', t_n);

    % Time axis for plotting (if needed)
    t_total = 1;                        % total duration (s)
    N = length(omega_m_array);
    T = linspace(0, t_total, N);

    % Angle tracking plots
    h7 = figure();
    plot(t, y(:,1)*180/pi, 'LineWidth', 2); hold on;
    plot(t, arrayfun(@ankle_trajectory_real, t)*180/pi, '--r', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Angle (°)');
    title('Ankle Joint Angle Tracking');
    legend('Actual Angle', 'Desired Trajectory');
    grid on;

    h8 = figure();
    plot(t, y(:,5)*180/pi, 'LineWidth', 2); hold on;
    plot(t, arrayfun(@toe_trajectory_real, t)*180/pi, '--r', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Angle (°)');
    title('Toe Joint Angle Tracking');
    legend('Actual Angle', 'Desired Trajectory');
    grid on;

    fprintf('Controller call count: %d\n', length(t));
    fprintf('Estimated average control frequency: %.2f Hz\n', length(t) / 1.1);
    count = evalin('base', 'spring_both_satisfied_count');
    fprintf('spring_both_satisfied_count = %d\n', count);
    count_f = evalin('base', 'spring_not_satisfied_count');
    fprintf('spring_not_satisfied_count = %d\n', count_f);
end

% ----------------------------
% Controller (runs at 1 kHz)
% ----------------------------
function [omega_m, tau_pid_t] = controller(t, y, Kp_a, Ki_a, Kd_a, Kp_t, Ki_t, Kd_t, dt)
    % persistent prev_de_a
    % if isempty(prev_de_a), prev_de_a = 0; end

    % Current states
    theta_a = y(1);
    omega_a = y(2);
    theta_t = y(5);
    omega_t = y(6);
    integral_e_a = y(7);
    integral_e_t = y(8);
    
    % References
    theta_a_ref = ankle_trajectory_real(t);
    omega_a_ref = ankle_angular_velocity_real(t);
    theta_t_ref = toe_trajectory_real(t);
    omega_t_ref = toe_angular_velocity_real(t);
    
    % Ankle PID
    e_a  = theta_a_ref - theta_a;
    de_a = omega_a_ref - omega_a;
    integral_e_a = integral_e_a + e_a * dt;
    omega_m_max = 50; % M3508 maximum (rad/s equivalent)

    omega_m = Kp_a*e_a + Ki_a*integral_e_a + Kd_a*de_a;
    omega_m = sign(omega_m) * min(abs(omega_m), omega_m_max);
    
    % Toe PID
    e_t  = theta_t_ref - theta_t;
    de_t = omega_t_ref - omega_t;
    integral_e_t = integral_e_t + e_t * dt;
    tau_pid_t_max = 1.5; % M2006 max torque works well at ~1.5
    tau_pid_t = Kp_t*e_t + Ki_t*integral_e_t + Kd_t*de_t;
    tau_pid_t = sign(tau_pid_t) * min(abs(tau_pid_t), tau_pid_t_max);

    fprintf('\n---------- @ t=%.3f ----------\n',t);
    fprintf(['t=%.3f, e_a=%.4f, int_e_a=%.4f, de_a=%.4f, ', ...
             'Kp_a*e_a=%.4f, Ki_a*int=%.4f, Kd_a*de_a=%.4f\n'], ...
            t, e_a, integral_e_a, de_a, Kp_a*e_a, Ki_a*integral_e_a, Kd_a*de_a);
    fprintf(['t=%.3f, e_t=%.4f, int_e_t=%.4f, de_t=%.4f, ', ...
             'Kp_t*e_t=%.4f, Ki_t*int=%.4f, Kd_t*de_t=%.4f\n'], ...
            t, e_t, integral_e_t, de_t, Kp_t*e_t, Ki_t*integral_e_t, Kd_t*de_t);
end

% ----------------------------
% Dynamics (fixed-step version)
% ----------------------------
function dydt = dynamics_v7(t, y, I_a_total, I_t, ...
    k1_1, k1_2, n, p, k3, n_t, b_a, b_t, b_m, eta, k2, ...
    omega_m, tau_pid_t, M_f_a, M_f_t, m_a)
    
    % States
    theta_a = y(1);
    omega_a = y(2);
    x_m     = y(3);
    theta_t = y(5);
    omega_t = y(6);

    % Speed-dependent ankle damping
    b_a_actual = damping_coefficient(abs(omega_a));
    
    % Geometry (same as your original code)
    d_r = 5.6/100;
    d_d = 1.55/100;
    l_k = 23/100;
    l_t = 7.5/100;
    h_a = 7/100;
    h_m = 3/100;
    h_t = 1.3/100;
    h_f = 18.8/100;
    d_a = 7.15/100;
    h_s = 11.8/100;
    l_k3 = 2.54/100;
    h_k3 = 2.54/100;
    d_k3 = 4.5/100; % was 4.5
    t_k1 = h_m;
    d_k1_1 = 2.5/100; %2.5
    l_k1_1 = 5.08/100;  %4
    d_k1_2 = 10.72/100; %10.4
    l_k1_2 = 2.54/100;  %3.2
    l_g1 = 3.2/100;
    l_g2 = 1.225/100;
    l_g3 = 0.6/100; %0.7
    limit = 3.75/100;
    
    % Ankle
    O1 = [0; -h_s];

    % Foot plate
    foot_pts = [-d_r-d_d, -h_f; -d_r-d_d+l_k, -h_f]';
    rot_a = [cos(theta_a), -sin(theta_a); sin(theta_a), cos(theta_a)];
    foot_rot = rot_a * (foot_pts - O1) + O1;

    % Left end of foot plate
    P0 = [-d_r-d_d; -h_f];

    % Spring k1 — plantarflexion side
    P1 = [-d_r-d_d+l_k; -h_f];
    dir = (P1 - P0) / norm(P1 - P0);
    P_local1_1 = P0 + dir * (d_k1_1+d_a);
    % Foot-side left endpoint of k1 (plantarflexion)
    P_global1_1 = rot_a * (P_local1_1 - O1) + O1;

    P_f1_x = P_global1_1(1) - t_k1*sin(theta_a);
    P_f1_y = P_global1_1(2) + t_k1*cos(theta_a);
    P_f1 = [P_f1_x; P_f1_y];
    
    % Spring k1 — dorsiflexion side
    P_local1_2 = P0 + dir * (d_k1_2+d_a);
    P_global1_2 = rot_a * (P_local1_2 - O1) + O1;

    P_f2_x = P_global1_2(1) - t_k1*sin(theta_a);
    P_f2_y = P_global1_2(2) + t_k1*cos(theta_a);
    P_f2 = [P_f2_x; P_f2_y];

    % Ankle rod contact point on foot
    P_local2 = P0 + dir * d_a;
    P_global2 = rot_a * (P_local2 - O1) + O1;

    % Transmission pivot
    P_t = [0.00;-(h_f-h_m)];
    
    % Screw endpoints (decomposed)
    P_m_x = P_t(1) + l_g1*cos(theta_a);
    P_m_y = P_t(2) + l_g1*sin(theta_a);
    P_m1 = [P_m_x;P_m_y];
    
    P_m_x = P_t(1) + (x_m+l_g1)*cos(theta_a);
    P_m_y = P_t(2) + (x_m+l_g1)*sin(theta_a);
    P_m2 = [P_m_x;P_m_y];
    
    % Nut seat left end (right endpoint for k1 plantarflexion)
    P_m_x = P_t(1) + (x_m+l_g1+l_g2)*cos(theta_a);
    P_m_y = P_t(2) + (x_m+l_g1+l_g2)*sin(theta_a);
    P_m3 = [P_m_x;P_m_y];

    % Nut seat right end (left endpoint for k1 dorsiflexion)
    P_m_x = P_t(1) + (x_m+l_g1+l_g2+l_g3)*cos(theta_a);
    P_m_y = P_t(2) + (x_m+l_g1+l_g2+l_g3)*sin(theta_a);
    P_m4 = [P_m_x;P_m_y];
 
    spring_vec1  = P_m3 - P_f1;
    spring_len1  = norm(spring_vec1);
    spring_vec2  = P_m4 - P_f2;
    spring_len2  = norm(spring_vec2);

    S_L = norm(P_f1 - P_f2);
    S_H = norm(P_m3 - P_m4);

    if spring_len1 <= (5.08/100) || spring_len2 <= (2.54/100)
        % If both conditions satisfied, increment the counter
        if spring_len1 <= (5.08/100) && spring_len2 <= (2.54/100)
            count = evalin('base', 'spring_both_satisfied_count');
            assignin('base', 'spring_both_satisfied_count', count + 1);
        end
        torque1_1 = 0;
        torque1_2 = 0;

        if spring_len1 <= (5.08/100) % original rest length 4 cm
            spring_vec1  = P_m3 - P_f1;
            spring_len1  = norm(spring_vec1);
            spring_dir1  = spring_vec1/ spring_len1;
            F_spring1_1  = k1_1 * (l_k1_1 - spring_len1) * spring_dir1;
            F1_1 = norm(F_spring1_1);
            r1_1 = P_f1 - O1; 
            torque1_1 = -cross2D(r1_1, F_spring1_1);

            fprintf(['\n--- Spring Debug (plantarflexion) @ t=%.3f ---\n', ...
            'spring_vec = [%.8f, %.8f]\n', ...
            'spring_len = %.8f\n', ...
            'spring_dir = [%.4f, %.4f]\n', ...
            'F_spring1_1 = [%.4f, %.4f], norm = %.4f\n', ...
            'r1_1 = [%.4f, %.4f]\n', ...
            'torque1 = %.4f\n', ...
            'P_f1 = [%.4f, %.4f], P_m3 = [%.4f, %.4f]\n', ...
            'theta_a = %.4f\n'], ...
            t, spring_vec1(1), spring_vec1(2), spring_len1, ...
            spring_dir1(1), spring_dir1(2), F_spring1_1(1), F_spring1_1(2), F1_1, ...
            r1_1(1), r1_1(2), torque1_1, ...
            P_f1(1), P_f1(2), P_m3(1), P_m3(2), theta_a);
            fprintf('t=%.3f, left', t);
            assignin('base', 'F1_array',  [evalin('base','F1_array');  F1_1]);
            assignin('base', 'F1_1array', [evalin('base','F1_1array'); F1_1]);
            assignin('base', 'spring_len1_array', [evalin('base','spring_len1_array'); spring_len1]);
        end
        
        if spring_len2 <= (2.54/100) % original rest length 3.2 cm
            spring_vec2  = P_m4 - P_f2;
            spring_len2  = norm(spring_vec2);
            spring_dir2  = spring_vec2 / spring_len2;
            F_spring1_2  = k1_2 * (l_k1_2 - spring_len2) * spring_dir2;
            F1_2 = norm(F_spring1_2);
            r1_2 = P_f2 - O1;
            torque1_2 = -cross2D(r1_2, F_spring1_2);

            fprintf(['\n--- Spring Debug (dorsiflexion) @ t=%.3f ---\n', ...
            'spring_vec = [%.8f, %.8f]\n', ...
            'spring_len = %.8f\n', ...
            'spring_dir = [%.4f, %.4f]\n', ...
            'F_spring1_2 = [%.4f, %.4f], norm = %.4f\n', ...
            'r1_2 = [%.4f, %.4f]\n', ...
            'torque1 = %.4f\n', ...
            'P_f2 = [%.4f, %.4f], P_m4 = [%.4f, %.4f]\n', ...
            'theta_a = %.4f\n'], ...
            t, spring_vec2(1), spring_vec2(2), spring_len2, ...
            spring_dir2(1), spring_dir2(2), F_spring1_2(1), F_spring1_2(2), F1_2, ...
            r1_2(1), r1_2(2), torque1_2, ...
            P_f2(1), P_f2(2), P_m4(1), P_m4(2), theta_a);
            fprintf('t=%.3f, right', t);
            assignin('base', 'F1_array',  [evalin('base','F1_array');  F1_2]);
            assignin('base', 'F1_2array', [evalin('base','F1_2array'); F1_2]);
            assignin('base', 'spring_len2_array', [evalin('base','spring_len2_array'); spring_len2]);
        end

        torque1 = torque1_1 + torque1_2;
    else
        torque1 = 0;
        warning('No spring torque applied at t=%.4f, x_m=%.4f, spring_len1=%.8f, spring_len2=%.8f, theta_a=%.4f\n', ...
            t, x_m, spring_len1, spring_len2, theta_a);
        count_f = evalin('base', 'spring_not_satisfied_count');
        assignin('base', 'spring_not_satisfied_count', count_f + 1);
        assignin('base', 'spring_not_satisfied_time', t);
    end

    assignin('base', 'torque_t1_array', [evalin('base','torque_t1_array'); torque1]);

    fprintf('\n');
    fprintf('t=%.3f, omega_m=%.4f', t, omega_m);
    fprintf('t=%.3f, x_m=%.4f', t, x_m);
    fprintf('t=%.3f, S_L=%.8f', t, S_L);
    fprintf('t=%.3f, S_H=%.8f\n', t, S_H);

    % Heel spring k3
    P_top = [-d_r-d_d+d_k3; -(h_f-h_k3)];
    local_s3 = [-d_r-d_d+d_k3; -h_a];
    P_bot = rot_a * local_s3 + O1;
    
    % Spring k3 geometry
    spring_vec3 = P_top - P_bot;
    spring_len3 = norm(spring_vec3);
    
    % Gravity torque (ankle-linked structure)
    tau_ankle_g = cos(theta_a)*0.05*m_a;

    % External torques
    toe_ankle = toe_ankle_ground_reaction_torque(t);
    tau_GRF   = ground_reaction_torque(t);

    % Total ankle torque (with efficiencies and damping/friction)
    total_torque_a = torque1 * eta.spring + tau_GRF + toe_ankle + tau_ankle_g ...
                   - b_a_actual * omega_a * eta.damping - M_f_a * sign(omega_a);
    pure_torque_a  = torque1 - b_a_actual * omega_a;

    if spring_len3 >= l_k3
        spring_dir3   = spring_vec3 / spring_len3;
        F_initial3    = 12.9 * spring_dir3; % preload
        F_spring3     = k3 * (spring_len3 - l_k3) * spring_dir3;
        F3            = norm(F_spring3);
        r3            = P_bot - O1;
        torque3       = cross2D(r3, F_spring3);
        torque_initial3 = cross2D(r3, F_initial3);
        total_torque_a = total_torque_a + (torque3 + torque_initial3)*eta.spring;
        pure_torque_a  = pure_torque_a + torque3;
        
        fprintf('t=%.3f, torque3=%.4f, spring_len3=%.4f, F3=%.4f, tau_pid_t=%.4f, omega_t=%.4f\n', ...
            t, torque3, spring_len3, F3, tau_pid_t, omega_t);
        assignin('base', 'torque_t3_array', [evalin('base','torque_t3_array'); torque3]);
    end
    assignin('base', 'torque_a_array', [evalin('base','torque_a_array'); total_torque_a]);

    % Toe torque
    torque_spring_t = -k2 * (theta_t - 0);
    damping_joint_t = -b_t * omega_t;
    damping_motor_t = -(b_m / n_t^2) * omega_t; % good motor → ignore if desired
    tau_toe_GRF     = toe_ground_reaction_torque(t);

    total_torque_t = tau_pid_t * eta.motor / n_t * eta.gear ...
                   + torque_spring_t * eta.spring ...
                   + damping_joint_t + tau_toe_GRF ...
                   - M_f_t * sign(omega_t);
    pure_torque_t  = tau_pid_t / n_t + torque_spring_t + damping_joint_t;
    assignin('base', 'torque_t_array', [evalin('base','torque_t_array'); total_torque_t]);
    
    % Angle limits (leave margin)
    theta_a_min = deg2rad(-20);
    theta_a_max = deg2rad(20);
    theta_t_min = deg2rad(-5);
    theta_t_max = deg2rad(37.5);
    
    % Hard limits for ankle
    if theta_a < theta_a_min
        y(1) = theta_a_min; omega_a = 0;
    elseif theta_a > theta_a_max
        y(1) = theta_a_max; omega_a = 0;
    end
    
    % Hard limits for toe
    if theta_t < theta_t_min
        y(5) = theta_t_min; omega_t = 0;
    elseif theta_t > theta_t_max
        y(5) = theta_t_max; omega_t = 0;
    end
    
    % Screw position derivative (lead p, with efficiencies)
    dx_m = omega_m * eta.motor * n * eta.gear * p * eta.screw;
    fprintf('t=%.3f, dx_m=%.4f\n', t, dx_m);

    % Screw travel limits (limit the derivative, not the state)
    x_min = 0; x_max = limit;
    if y(3) <= x_min && dx_m < 0
        dx_m = 0;
    elseif y(3) >= x_max && dx_m > 0
        dx_m = 0;
    end

    % Derivatives
    dydt = zeros(8,1);
    dydt(1) = omega_a;
    dydt(2) = total_torque_a / I_a_total;
    dydt(3) = dx_m;
    dydt(5) = omega_t;
    dydt(6) = total_torque_t / I_t;
    dydt(7) = (ankle_trajectory_real(t) - theta_a); % error integral
    dydt(8) = (toe_trajectory_real(t) - theta_t);   % error integral

    % Log to base
    assignin('base', 'spring_len3_array', [evalin('base','spring_len3_array'); spring_len3]);
    assignin('base', 'x_m_array', [evalin('base','x_m_array'); x_m]);
    assignin('base', 'omega_m_array', [evalin('base','omega_m_array'); omega_m]);
    assignin('base', 'omega_t_array', [evalin('base','omega_t_array'); omega_t]);
    assignin('base', 'tau_pid_t_array', [evalin('base','tau_pid_t_array'); tau_pid_t]);
end

function theta_ref = ankle_trajectory_real(t)
    % Able-bodied ankle angle trajectory (black solid line)
    % Input: t ∈ [0, 1] (s)
    % Output: theta_ref (rad)

    % Map t ∈ [0,1] → %GC ∈ [0,100]
    gc = t * 100;

    % Data (deg) estimated from plot
    x_gc =        [0   5  10  15  20  25  30  35  40   45    50   55  60   65    70  75   80   85   90  95 100 105 110];
    y_angle_deg = [-5 -7.5 -5 -2.5  0 1.25 2.5   5   6.5  8.5  10    5   -5  -12.5 -10  -5   -1    0   -2 -3.5 -5  -5  -5];
                 
    % Interpolate & convert to radians
    y_angle_rad = deg2rad(y_angle_deg);
    theta_ref   = interp1(x_gc, y_angle_rad, gc, 'pchip');

    % Out-of-range protection
    theta_ref(gc < 0 | gc > 110) = deg2rad(-5);
end

function theta_ref = ankle_trajectory_step(t)
    % Step input ankle trajectory
    % Input: t ∈ [0, 1] (s)
    % Output: theta_ref (rad)
    step_time = 0;
    theta_low_deg  = 0;
    theta_high_deg = 1;
    theta_low  = deg2rad(theta_low_deg);
    theta_high = deg2rad(theta_high_deg);
    theta_ref  = theta_low * ones(size(t));
    theta_ref(t > step_time) = theta_high;
end

function theta_ref = toe_trajectory_real(t)
    % Toe angle trajectory estimated from the first (blue) curve
    % Input: t ∈ [0,1] (s)
    % Output: theta_ref (rad)

    % Map t → %GC
    gc = t * 100;

    % Estimated blue-curve data (deg)
    x_gc =        [0  5 10 15 20 25 30 35 40 45 50 55   60 65 70 75    80   85 90  95 100 105 110];
    y_angle_deg = [12 8 4  2  0 1.75 2 3.5  6  9 15 27.5 37 20 10 10.25 10.5 11 11.25 11.5 12 12 12];

    % Interpolate & convert
    y_angle_rad = deg2rad(y_angle_deg);
    theta_ref   = interp1(x_gc, y_angle_rad, gc, 'pchip');

    % Out-of-range protection
    theta_ref(gc < 0 | gc > 110) = deg2rad(12);
end

function omega_ref = ankle_angular_velocity_real(t)
    % Numerical differentiation of ankle reference (rad/s)
    dt = 0.001;
    if t < dt
        theta_now  = ankle_trajectory_real(t);
        theta_next = ankle_trajectory_real(t + dt);
        omega_ref  = (theta_next - theta_now) / dt;
    else
        theta_now  = ankle_trajectory_real(t);
        theta_prev = ankle_trajectory_real(t - dt);
        omega_ref  = (theta_now - theta_prev) / dt;
    end
end

function omega_ref = ankle_angular_velocity_step(t)
    % Numerical differentiation of the step trajectory (rad/s)
    dt = 0.001;
    if t < dt
        theta_now  = ankle_trajectory_step(t);
        theta_next = ankle_trajectory_step(t + dt);
        omega_ref  = (theta_next - theta_now) / dt;
    else
        theta_now  = ankle_trajectory_step(t);
        theta_prev = ankle_trajectory_step(t - dt);
        omega_ref  = (theta_now - theta_prev) / dt;
    end
end

function omega_ref = toe_angular_velocity_real(t)
    % Numerical differentiation of toe reference (rad/s)
    dt = 0.001;
    if t < dt
        theta_now  = toe_trajectory_real(t);
        theta_next = toe_trajectory_real(t + dt);
        omega_ref  = (theta_next - theta_now) / dt;
    else
        theta_now  = toe_trajectory_real(t);
        theta_prev = toe_trajectory_real(t - dt);
        omega_ref  = (theta_now - theta_prev) / dt;
    end
end

function z = cross2D(a, b)
    % 2D cross product → scalar z-component
    z = a(1)*b(2) - a(2)*b(1);
end

function visualize_motion_v7(t, y)
    % Geometry (meters). Coordinate frame:
    % Origin at the center of the ground contact plane.
    % +x to the right, +y up. Foot plate thickness ignored.
    d_r = 5.6/100;   % half width of contact plane
    d_d = 1.55/100;  % heel-to-plane-left distance
    l_k = 23/100;    % foot plate length (without toe)
    l_t = 7.5/100;   % toe length
    h_a = 7/100;     % ankle height from foot bottom
    h_m = 3/100;     % transmission pivot height from foot bottom
    h_t = 1.3/100;   % toe joint height from foot bottom
    h_f = 18.8/100;  % total foot height from bottom
    d_a = 7.15/100;  % horizontal ankle position from heel
    h_s = 11.8/100;  % ankle to contact plane vertical
    l_k3 = 2.54/100; % heel spring k3 rest length
    h_k3 = 2.54/100; % k3 top vertical offset
    d_k3 = 4.5/100;  % k3 horizontal offset from heel
    h_c = 0/100;     % coupler vertical offset (unused)
    l_c = 0/100;     % coupler equivalent length (unused)

    % Design-dependent points
    d_k1_1 = 2.5/100;   % k1 plantarflexion right-end foot-side horizontal offset
    l_k1_1 = 5.08/100;  % k1 plantarflexion rest length
    d_k1_2 = 10.72/100; % k1 dorsiflexion left-end foot-side horizontal offset
    l_k1_2 = 2.54/100;  % k1 dorsiflexion rest length
    t_k1   = h_m;       % vertical offset (spring seat aligned with screw & coupler)

    l_g1 = 3.2/100;   % screw device left horizontal length
    l_g2 = 1.225/100; % screw device mid horizontal (nut-seat offset)
    l_g3 = 0.6/100;   % screw device right horizontal (nut seat thickness)

    for i = 1:10:length(t)
        theta_a = y(i,1);
        x_m     = y(i,3);
        theta_t = y(i,5);

        clf; hold on;
        
        % Upper rod above ankle
        plot([0, 0], [0, -h_s], 'k-', 'LineWidth', 3);

        % Ankle joint
        O1 = [0; -h_s];
        plot(O1(1), O1(2), 'ko', 'MarkerFaceColor', 'w'); 
        draw_circle(O1, 0.01, 'k', 1);
        draw_circle(O1, 0.015, 'k', 1);
        
        % GRF contact plane
        plot([-d_r, d_r], [0, 0], 'b-', 'LineWidth', 3);

        % Foot plate
        foot_pts = [-d_r-d_d, -h_f; -d_r-d_d+l_k, -h_f]';
        rot_a = [cos(theta_a), -sin(theta_a); sin(theta_a), cos(theta_a)];
        foot_rot = rot_a * (foot_pts - O1) + O1;
        plot(foot_rot(1,:), foot_rot(2,:), 'b-', 'LineWidth', 3);

        % Left end of foot plate
        P0 = [-d_r-d_d; -h_f];

        % k1 plantarflexion side
        P1 = [-d_r-d_d+l_k; -h_f];
        dir = (P1 - P0) / norm(P1 - P0);
        P_local1_1 = P0 + dir * (d_k1_1+d_a);
        P_global1_1 = rot_a * (P_local1_1 - O1) + O1;

        P_f1_x = P_global1_1(1) - t_k1*sin(theta_a);
        P_f1_y = P_global1_1(2) + t_k1*cos(theta_a);
        P_f1 = [P_f1_x; P_f1_y];

        plot([P_global1_1(1), P_f1(1)], [P_global1_1(2), P_f1(2)], 'k-', 'LineWidth', 3);
        
        % k1 dorsiflexion side
        P_local1_2 = P0 + dir * (d_k1_2+d_a);
        P_global1_2 = rot_a * (P_local1_2 - O1) + O1;

        P_f2_x = P_global1_2(1) - t_k1*sin(theta_a);
        P_f2_y = P_global1_2(2) + t_k1*cos(theta_a);
        P_f2 = [P_f2_x; P_f2_y];

        plot([P_global1_2(1), P_f2(1)], [P_global1_2(2), P_f2(2)], 'k-', 'LineWidth', 3);
        
        % Ankle rod location on foot
        P_local2 = P0 + dir * d_a;
        P_global2 = rot_a * (P_local2 - O1) + O1;
        plot([P_global2(1), 0], [P_global2(2), -h_s], 'k-', 'LineWidth', 3);
        
        % Transmission pivot
        P_t = [0.00;-(h_f-h_m)];
        plot(0.00, -(h_f-h_m), 'ko', 'MarkerFaceColor', 'w');
        plot([P_t(1), O1(1)], [P_t(2), O1(2)], 'g-', 'LineWidth', 3);

        % Screw segments
        P_m_x = P_t(1) + l_g1*cos(theta_a);
        P_m_y = P_t(2) + l_g1*sin(theta_a);
        P_m1 = [P_m_x;P_m_y];
        
        P_m_x = P_t(1) + (x_m+l_g1)*cos(theta_a);
        P_m_y = P_t(2) + (x_m+l_g1)*sin(theta_a);
        P_m2 = [P_m_x;P_m_y];
        
        P_m_x = P_t(1) + (x_m+l_g1+l_g2)*cos(theta_a);
        P_m_y = P_t(2) + (x_m+l_g1+l_g2)*sin(theta_a);
        P_m3 = [P_m_x;P_m_y];

        P_m_x = P_t(1) + (x_m+l_g1+l_g2+l_g3)*cos(theta_a);
        P_m_y = P_t(2) + (x_m+l_g1+l_g2+l_g3)*sin(theta_a);
        P_m4 = [P_m_x;P_m_y];
     
        plot([P_t(1), P_m1(1)], [P_t(2), P_m1(2)], 'k-', 'LineWidth', 3);
        plot([P_m1(1), P_m2(1)], [P_m1(2), P_m2(2)], 'y-', 'LineWidth', 3);
        plot([P_m2(1), P_m3(1)], [P_m2(2), P_m3(2)], 'k-', 'LineWidth', 3);
        plot([P_f1(1), P_m3(1)], [P_f1(2), P_m3(2)], 'r--', 'LineWidth', 3);
        plot([P_m3(1), P_m4(1)], [P_m3(2), P_m4(2)], 'k-', 'LineWidth', 3);
        plot([P_m4(1), P_f2(1)], [P_m4(2), P_f2(2)], 'r--', 'LineWidth', 3);

        % Spring seats
        P_f3_x = P_m3(1) + t_k1*sin(theta_a);
        P_f3_y = P_m3(2) - t_k1*cos(theta_a);
        P_f3 = [P_f3_x; P_f3_y];
        plot([P_m3(1), P_f3(1)], [P_m3(2), P_f3(2)], 'c-', 'LineWidth', 3);

        P_f4_x = P_m4(1) + t_k1*sin(theta_a);
        P_f4_y = P_m4(2) - t_k1*cos(theta_a);
        P_f4 = [P_f4_x; P_f4_y];
        plot([P_m4(1), P_f4(1)], [P_m4(2), P_f4(2)], 'c-', 'LineWidth', 3);

        % Spring endpoints
        plot(P_m3(1), P_m3(2), 'ko', 'MarkerFaceColor', 'm');
        plot(P_m4(1), P_m4(2), 'ko', 'MarkerFaceColor', 'm');
        plot(P_f1(1), P_f1(2), 'ko', 'MarkerFaceColor', 'm');
        plot(P_f2(1), P_f2(2), 'ko', 'MarkerFaceColor', 'm');

        % Heel spring k3
        P_top = [-d_r-d_d+d_k3; -(h_f-h_k3)];
        local_s3 = [-d_r-d_d+d_k3; -h_a];
        P_bot = rot_a * local_s3 + O1;
        plot([P_top(1), P_bot(1)], [P_top(2), P_bot(2)], 'r--', 'LineWidth', 2);
        
        % k3 top vertical bar
        plot([P_top(1), P_top(1)], [P_top(2), 0], 'k-', 'LineWidth', 3);
        
        plot(P_bot(1), P_bot(2), 'ko', 'MarkerFaceColor', 'm');
        plot(P_top(1), P_top(2), 'ko', 'MarkerFaceColor', 'm');
        
        % Toe joint
        O2_local  = [-d_r-d_d+l_k; -h_f+h_t];
        O2_local2 = [-d_r-d_d+l_k; -h_f];
        O2   = rot_a * (O2_local  - O1) + O1;
        O2_2 = rot_a * (O2_local2 - O1) + O1;
        draw_circle(O2, 0.01, 'k', 1);

        toe_length = l_t;
        rot_t = [cos(theta_a+theta_t), -sin(theta_a+theta_t); ...
                 sin(theta_a+theta_t),  cos(theta_a+theta_t)];
        toe_vec = rot_t * [toe_length; 0];
        toe_end = O2_2 + toe_vec;
        
        plot([O2_2(1), O2(1)],     [O2_2(2), O2(2)],     'k-', 'LineWidth', 3);
        plot([O2_2(1), toe_end(1)], [O2_2(2), toe_end(2)], 'b-', 'LineWidth', 3);
        plot(O2(1), O2(2), 'ko', 'MarkerFaceColor', 'w');
        draw_circle(O2, 0.006, 'r', 1);

        axis equal;
        axis([-0.1 0.3 -0.3 0.05]);
        title(sprintf('Time %.2f s', t(i)));
        xlabel('X (m)');
        ylabel('Y (m)');
        grid on;
        pause(0.02);
    end
end

function draw_circle(center, radius, color, linewidth)
    theta = linspace(0, 2*pi, 50);
    x = center(1) + radius * cos(theta);
    y = center(2) + radius * sin(theta);
    plot(x, y, 'Color', color, 'LineWidth', linewidth, 'LineStyle', '--');
end

% GRF torque about the ankle during 0–60% gait; heel friction torque neglected.
function tau_GRF = ground_reaction_torque(t)
    feet_length = 0.3;  % m
    Body_weight = 60;   % kg
    T_stance = 0.6;     % s (stance duration)

    % COP x-position as % foot length (from literature figure)
    r_x = [10 15 22.5 27.5 30 32.5 37.5 42 45 51 56 60 65 70 72.5 74.5 76 78 81.5 84 91];
    x_old = linspace(0, T_stance, length(r_x));

    % Interpolate to 10 points
    t_interp = linspace(0, T_stance, 10);
    r_x_interp = interp1(x_old, r_x, t_interp, 'linear');
    r_x_net = r_x_interp - 25;             % ankle at 25% of foot length
    r_x_actual = r_x_net * feet_length/100;

    % Vertical GRF in BW (from literature figure)
    F_y = [0 9 12 11.5 10.5 9.5 9 9.5 10.5 11 10 4 0];
    F_y_interp = interp1(linspace(0, T_stance, length(F_y)), F_y, t_interp);
    F_y_actual = F_y_interp * Body_weight;

    % Torque r × F (scalar in 2D)
    tau_half = r_x_actual .* F_y_actual;

    % Output at requested times
    tau_GRF = zeros(size(t));
    idx = t <= T_stance;
    tau_GRF(idx) = interp1(t_interp, tau_half, t(idx), 'linear', 0);
end

% Toe GRF torque (50–60% gait) 
function tau_toe_GRF = toe_ground_reaction_torque(t)
    feet_length = 0.3;  % m
    Body_weight = 60;   % kg
    T_start = 0.5;      % toe support start
    T_stance = 0.1;     % duration (0.5–0.6 s)

    t_interp = linspace(T_start, T_start + T_stance, 10);

    % COP in toe region (as % foot length); ankle at ~75%
    r_x = [77 81.5 91];
    t_r = linspace(T_start, T_start + T_stance, length(r_x));
    r_x_interp = interp1(t_r, r_x, t_interp, 'linear');

    r_x_net = r_x_interp - 75;
    r_x_actual = r_x_net * feet_length / 100;

    % Vertical GRF in BW (reduced at segment start)
    F_y = [1.5 3.5 0.5];
    t_f = linspace(T_start, T_start + T_stance, length(F_y));
    F_y_interp = interp1(t_f, F_y, t_interp, 'linear');
    F_y_actual = F_y_interp * Body_weight;

    tau_toe_half = r_x_actual .* F_y_actual;

    tau_toe_GRF = zeros(size(t));
    idx = (t >= T_start) & (t <= T_start + T_stance);
    t_local = t(idx) - T_start;
    tau_toe_GRF(idx) = interp1(linspace(0, T_stance, length(tau_toe_half)), ...
                               tau_toe_half, t_local, 'linear', 0);
end

% (Unused) Estimated leg torque about the ankle over gait
function tau_leg_GRF = leg_ground_reaction_torque(t)
    Body_weight = 60; % kg
    T_stance = 1;     % s
    t_interp = linspace(0, T_stance, 20);

    r_x = [-5 -2.5 0 1 2 4 6 8 10 12 13 14 15 0 0 0 0 0 0 0 0]/100;
    F_y = [0 9 12 11.5 10.5 9.5 9 9.5 10.5 12 11 6 1 0 0 0 0 0 0 0]; % BW
    r_x_interp = interp1(linspace(0, T_stance, length(r_x)), r_x, t_interp);
    F_y_interp = interp1(linspace(0, T_stance, length(F_y)), F_y, t_interp);
    F_y_actual = F_y_interp * Body_weight;

    tau_leg_half = r_x_interp .* F_y_actual;
    tau_leg_GRF = zeros(size(t));
    idx = t <= T_stance;
    tau_leg_GRF(idx) = interp1(t_interp, tau_leg_half, t(idx), 'linear', 0);
end

% Toe → ankle reaction moment during 50–60% gait (literature-based shape)
function toe_ankle = toe_ankle_ground_reaction_torque(t)
    T_start = 0.1; 
    T_stance = 0.5;  
    Body_weight = 60; 

    % Example profile scaled by BW
    M_toe_ankle = Body_weight * [0 0.005 0.01 0.015 0.025 0.04 0.07 0.1 0.125 0.11 0.025];
    t_interp = linspace(0, T_stance, length(M_toe_ankle));

    toe_ankle = zeros(size(t));
    idx = (t >= T_start) & (t <= T_start + T_stance);
    t_local = t(idx) - T_start;
    toe_ankle(idx) = interp1(t_interp, M_toe_ankle, t_local, 'linear', 0);
end

function tau = pyStyleCross2D(a, b)
    % Equivalent to np.cross([a,0],[b,0]) z-component
    cross_3d = cross([a(1), a(2), 0], [b(1), b(2), 0]);
    tau = cross_3d(3);
end

function ba = damping_coefficient(omega)
    % Speed-dependent damping model:
    %   ba(omega) = a * exp(b * omega), zeroed for very low speeds
    % Input:
    %   omega (rad/s), scalar or vector
    % Output:
    %   ba (N·m·s/rad), same size as omega
    a = 5.04;   % lower bound ~3
    b = -0.31;
    ba = a * exp(b * omega);
    ba(omega < 0.3) = 0;  % deadband near zero speed
end

