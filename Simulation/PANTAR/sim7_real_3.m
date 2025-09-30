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
    

    % Parameter definition:
    % Mass distribution guess: parts above the ankle < 1 kg; toe ~0.1 kg; total ~3 kg
    I_a = 0.01;      % Foot plate moment of inertia (kg·m^2) – approx. 2 kg rotating with ankle, + other parts maybe 2.5 kg
    I_t = 0.0001;    % Toe joint moment of inertia (kg·m^2)
    m_t = 0.1;       % Toe mass (kg)
    d = 0.155;       % Distance from toe joint to ankle joint (m)
    eta = 0.9;       % Efficiency factor η 
    
    % Additional inertia should be added for motors and other parts on the foot plate.
    % (This model targets the swing/stance phases excluding the transition
    % from full-foot contact to forefoot contact to final toe-off.)
    I_toe_at_ankle = I_t + m_t * d^2;
    I_a_total = I_a + I_toe_at_ankle;
    
    % If excluding that period, ankle selection need not include toe inertia (won’t rotate together).
    % I_a_total = I_a;

    % System 1 – Ankle
    k1_1 = 156920;   % Spring k1 (plantarflexion) stiffness – contributes ankle torque (left spring)
    k1_2 = 156920;   % Spring k1 (dorsiflexion) stiffness – contributes ankle torque (right spring)
    n = 1;           % Gear ratio of k1 system
    p_m_per_rev = 0.01; % Ball screw lead (m/rev). Note trade-offs: larger lead → lower torque capacity; smaller lead → slower response. Using 10 mm here.

    p = p_m_per_rev / (2 * pi);  % m/rad
    theta_a0 = deg2rad(0);       % Initial ankle angle

    % System 3 – Heel spring
    k3 = 2380;       % Spring k3 stiffness
    
    % System 2 – Toe
    n_t = 0.5;                 % Gear ratio of k2 system
    theta_t0 = deg2rad(11);    % Initial toe joint angle
    I_actual = 0;              % Measured motor current (external input in real system)
    k2 = 1.075;                % Spring k2 stiffness (N·m/rad)

    % Damping
    b_a = 0.05;       % Ankle damping N·m·s/rad
    b_t = 0.01;       % Toe joint damping N·m·s/rad
    b_m = 0.005;      % Motor-axis (toe) damping N·m·s/rad

    % Initial x_m depends on initial ankle angle
    x_m = 2.075/100;

    % Initial state:
    % State vector adds 4 entries for integral errors: ankle integral, last ankle error, toe integral, last toe error
    y0 = [theta_a0; 0; x_m; 0; theta_t0; 0; 0; 0]; 

    tspan = [0 1.15];
    % options = odeset('RelTol', 1e-8, 'AbsTol', 1e-8); % tighter solver tolerance (optional)

    [t, y] = ode45(@(t,y) dynamics_v7(t, y, ...
        I_a_total, I_t, k1_1, k1_2, n, p, k3, ...
        n_t, b_a, b_t, b_m, eta, I_actual, k2), tspan, y0);

    % Visualization
    h1 = figure();
    visualize_motion_v7(t, y);

    % Ankle tracking
    h2 = figure();
    plot(t, y(:,1)*180/pi, 'LineWidth', 2); hold on;
    plot(t, arrayfun(@ankle_trajectory_real, t)*180/pi, '--r', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    title('Ankle Angle Tracking (solid = actual, dashed = reference)');
    legend('Actual', 'Reference');
    grid on;
    
    % Toe tracking
    h3 = figure();
    plot(t, y(:,5)*180/pi, 'LineWidth', 2); hold on;
    plot(t, arrayfun(@toe_trajectory_real, t)*180/pi, '--r', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    title('Toe Angle Tracking (solid = actual, dashed = reference)');
    legend('Actual', 'Reference');
    grid on;
   

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

    % Actual angles
    theta1_actual = y(:,1);  % Ankle
    theta2_actual = y(:,5);  % Toe
    
    % Reference angles (via functions)
    theta1_desired = arrayfun(@ankle_trajectory_real, t);
    theta2_desired = arrayfun(@toe_trajectory_real, t);
    
    % Absolute error vectors (can switch to squared errors if preferred)
    ankle_error = abs(theta1_actual - theta1_desired);
    toe_error = abs(theta2_actual - theta2_desired);
    % ankle_error = (theta1_actual - theta1_desired).^2;
    % toe_error = (theta2_actual - theta2_desired).^2;
    
    % Accumulated absolute error
    ankle_error_sum = sum(ankle_error);
    toe_error_sum = sum(toe_error);
    
    % Print error results
    fprintf('Accumulated ankle error: %.4f rad\n', ankle_error_sum);
    fprintf('Accumulated toe error:   %.4f rad\n', toe_error_sum);

    % Error curves
    h4 = figure();
    plot(t, ankle_error, 'r', 'LineWidth', 2); hold on;
    plot(t, toe_error, 'b', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Error (rad)');
    legend('Ankle error', 'Toe error');
    title('Angle Error vs Time (Ankle & Toe)');
    grid on;

    % % Cumulative error over time (optional)
    % ankle_error_cumsum = cumsum(ankle_error);
    % toe_error_cumsum   = cumsum(toe_error);
    % h5 = figure();
    % plot(t, ankle_error_cumsum, 'r', 'LineWidth', 2); hold on;
    % plot(t, toe_error_cumsum, 'b', 'LineWidth', 2);
    % xlabel('Time (s)');
    % ylabel('Cumulative Error (rad)');
    % legend('Ankle cumulative error', 'Toe cumulative error');
    % title('Cumulative Error Growth (Ankle & Toe)');
    % grid on;
end

function dydt = dynamics_v7(t, y, I_a_total, I_t, ...
    k1_1,k1_2, n, p, k3, n_t, b_a, b_t, b_m, eta, I_actual, k2)

    theta_a = y(1); % Ankle angle
    omega_a = y(2); % Ankle angular velocity
    x_m = y(3);     % Ball-screw nut position
    % omega_m = y(4); % Motor angular velocity
    theta_t = y(5); % Toe angle
    omega_t = y(6); % Toe angular velocity
    integral_e_a = y(7); 
    integral_e_t = y(8);

    % Reference trajectories
    theta_a_ref = ankle_trajectory_real(t);
    theta_t_ref = toe_trajectory_real(t);
    omega_a_ref = ankle_angular_velocity_real(t);
    omega_t_ref = toe_angular_velocity_real(t);

    % --- Ankle PID → desired motor speed ---
    e_a = theta_a_ref - theta_a;
    % omega_a_ref often 0 if regulating to a setpoint
    de_a = omega_a_ref - omega_a;  % velocity error as derivative estimate

    Kp_a = 600; Ki_a = 1000; Kd_a = 6000;
    % omega_m = (Kp_a*e_a + Ki_a*integral_e_a);
    omega_m = (Kp_a*e_a + Ki_a*integral_e_a + Kd_a * de_a);
   
    % --- Toe PID ---
    e_t = theta_t_ref - theta_t;
    de_t = omega_t_ref - omega_t;  

    Kp_t = 800; Ki_t = 10; Kd_t = 10; % Ki_t = 11
    % tau_pid_t = Kp_t*e_t + Ki_t*integral_e_t + Kd_t*de_t;
    tau_pid_t = Kp_t*e_t + Ki_t*integral_e_t;

    % Torque-to-current
    k_t = 0.5;
    I_ref = tau_pid_t / k_t;
    
    % Inner current PI (placeholder for hardware current loop)
    % error_I = I_ref - I_actual;
    % voltage_out = PI_current_control(error_I);

    % Geometry (meters)
    d_r = 5.6/100;
    d_d = 1.55/100; % Heel to left edge of receiving plane
    l_k = 23/100;   % Footplate length (no toe)
    l_t = 7.5/100;  % Toe length
    h_a = 7/100;    % Ankle height (to sole)
    h_m = 3/100;    % Transmission pivot height (to sole)
    h_t = 1.3/100;  % Toe joint height (to sole)
    h_f = 18.8/100; % Total foot height (to sole)
    d_a = 7.15/100; % Horizontal ankle position from heel
    h_s = 11.8/100; % Vertical ankle-to-receiving-plane distance
    l_k3 = 4/100;   % Heel spring k3 free length (compression spring)
    h_k3 = 4/100;   % Vertical distance from k3 top to footplate
    d_k3 = 4.5/100; % Horizontal distance from k3 to heel
    h_c = 0/100;    % Vertical offset between transmission pivot and coupling (set 0 in new design)
    l_c = 0/100;    % Equivalent coupling length (ignored)
    limit = 3.75/100; % Ball screw stroke

    % Design-dependent variables
    d_k1_1 = 2.5/100;  % Spring k1 plantarflexion – left end position on footplate (from ankle)
    l_k1_1 = 4/100;    % Spring k1 plantarflexion – free length
    d_k1_2 = 10.4/100; % Spring k1 dorsiflexion – left end position on footplate (from ankle)
    l_k1_2 = 3.2/100;  % Spring k1 dorsiflexion – free length
    t_k1 = h_m;        % Vertical offset of k1 (aligned with screw & coupling)

    l_g1 = 3.2/100;  % Ball screw fixed horizontal length (left)
    l_g2 = 1.225/100;% Ball screw nut seat extra horizontal (middle)
    l_g3 = 0.7/100;  % Ball screw nut seat thickness (right)
   
    % Ankle joint
    O1 = [0; -h_s]; % Ankle coordinates

    % Footplate
    foot_pts = [-d_r-d_d, -h_f; -d_r-d_d+l_k, -h_f]';
    rot_a = [cos(theta_a), -sin(theta_a); sin(theta_a), cos(theta_a)];
    foot_rot = rot_a * (foot_pts - O1) + O1;

    % Left endpoint (initial)
    P0 = [-d_r-d_d; -h_f];

    % Spring 1 – plantarflexion
    P1 = [-d_r-d_d+l_k; -h_f];
    dir = (P1 - P0) / norm(P1 - P0);
    P_local1_1 = P0 + dir * (d_k1_1+d_a);
    % Spring 1 plantarflexion left endpoint on footplate
    P_global1_1 = rot_a * (P_local1_1 - O1) + O1;

    % Spring 1 plantarflexion – left endpoint (fixed on footplate)
    P_f1_x = P_global1_1(1) - t_k1*sin(theta_a);
    P_f1_y = P_global1_1(2) + t_k1*cos(theta_a);
    P_f1 = [P_f1_x; P_f1_y];
    
    % Spring 1 – dorsiflexion
    P1 = [-d_r-d_d+l_k; -h_f];
    dir = (P1 - P0) / norm(P1 - P0);
    P_local1_2 = P0 + dir * (d_k1_2+d_a);
    % Spring 1 dorsiflexion right endpoint on footplate
    P_global1_2 = rot_a * (P_local1_2 - O1) + O1;

    % Spring 1 dorsiflexion – right endpoint (fixed on footplate)
    P_f2_x = P_global1_2(1) - t_k1*sin(theta_a);
    P_f2_y = P_global1_2(2) + t_k1*cos(theta_a);
    P_f2 = [P_f2_x; P_f2_y];

    % Ankle rod position on footplate
    P_local2 = P0 + dir * d_a;
    P_global2 = rot_a * (P_local2 - O1) + O1;

    % Yellow pivot: transmission rotation point
    P_t = [0.00;-(h_f-h_m)];
    
    % Ball-screw endpoints
    % Fixed segment (left)
    P_m_x = P_t(1) + l_g1*cos(theta_a);
    P_m_y = P_t(2) + l_g1*sin(theta_a);
    P_m1 = [P_m_x; P_m_y];
    
    % Variable segment: nut left
    P_m_x = P_t(1) + (x_m+l_g1)*cos(theta_a);
    P_m_y = P_t(2) + (x_m+l_g1)*sin(theta_a);
    P_m2 = [P_m_x; P_m_y];
    
    % Variable: nut seat left end – right endpoint of spring 1 (plantarflexion)
    P_m_x = P_t(1) + (x_m+l_g1+l_g2)*cos(theta_a);
    P_m_y = P_t(2) + (x_m+l_g1+l_g2)*sin(theta_a);
    P_m3 = [P_m_x; P_m_y];

    % Variable: nut seat right end – left endpoint of spring 1 (dorsiflexion)
    P_m_x = P_t(1) + (x_m+l_g1+l_g2+l_g3)*cos(theta_a);
    P_m_y = P_t(2) + (x_m+l_g1+l_g2+l_g3)*sin(theta_a);
    P_m4 = [P_m_x; P_m_y];
 
    % Spring endpoints on nut seats (offset by t_k1)
    P_f3_x = P_m3(1) + t_k1*sin(theta_a);
    P_f3_y = P_m3(2) - t_k1*cos(theta_a);
    P_f3 = [P_f3_x; P_f3_y];

    P_f4_x = P_m4(1) + t_k1*sin(theta_a);
    P_f4_y = P_m4(2) - t_k1*cos(theta_a);
    P_f4 = [P_f4_x; P_f4_y];
    
    % Spring lengths
    spring_vec1 = P_m3 - P_f1;
    spring_len1 = norm(spring_vec1);
    spring_vec2 = P_m4 - P_f2;
    spring_len2 = norm(spring_vec2);

    S_L = norm(P_f1 - P_f2);
    S_H = norm(P_m3 - P_m4);

    if spring_len1 <= (4.01/100)
        % k1 plantarflexion torque (left spring)
        spring_vec = P_m3 - P_f1;
        spring_len = norm(spring_vec);
        spring_dir = spring_vec / spring_len;
        F_spring1_1 = k1_1 * (l_k1_1 - spring_len) * spring_dir;
        F1 = norm(F_spring1_1);
        r1_1 = P_f1 - O1; 
        % NOTE: sign differs from Python; MATLAB cross2D returns scalar z=r_x*F_y - r_y*F_x
        torque1 = -cross2D(r1_1, F_spring1_1); 

        fprintf(['\n--- Spring Debug (Plantarflexion) @ t=%.3f ---\n', ...
        'spring_vec = [%.8f, %.8f]\n', ...
        'spring_len = %.8f\n', ...
        'spring_dir = [%.4f, %.4f]\n', ...
        'F_spring1_1 = [%.4f, %.4f], norm = %.4f\n', ...
        'r1_1 = [%.4f, %.4f]\n', ...
        'torque1 = %.4f\n', ...
        'P_f1 = [%.4f, %.4f], P_m3 = [%.4f, %.4f]\n', ...
        'theta_a = %.4f\n'], ...
        t, spring_vec(1), spring_vec(2), spring_len, spring_dir(1), spring_dir(2), ...
        F_spring1_1(1), F_spring1_1(2), F1, r1_1(1), r1_1(2), torque1, ...
        P_f1(1), P_f1(2), P_m3(1), P_m3(2), theta_a);
        fprintf('t=%.3f, left',t);
    
    elseif spring_len2 <= (3.21/100)
        % k1 dorsiflexion torque (right spring)
        spring_vec = P_m4 - P_f2;
        spring_len = norm(spring_vec);
        spring_dir = spring_vec / spring_len;
        F_spring1_2 = k1_2 * (l_k1_2 - spring_len) * spring_dir;
        F1 = norm(F_spring1_2);
        r1_2 = P_f2 - O1; 
        % See note above re sign
        torque1 = -cross2D(r1_2, F_spring1_2); 

        fprintf(['\n--- Spring Debug (Dorsiflexion) @ t=%.3f ---\n', ...
        'spring_vec = [%.8f, %.8f]\n', ...
        'spring_len = %.8f\n', ...
        'spring_dir = [%.4f, %.4f]\n', ...
        'F_spring1_2 = [%.4f, %.4f], norm = %.4f\n', ...
        'r1_2 = [%.4f, %.4f]\n', ...
        'torque1 = %.4f\n', ...
        'P_f2 = [%.4f, %.4f], P_m4 = [%.4f, %.4f]\n', ...
        'theta_a = %.4f\n'], ...
        t, spring_vec(1), spring_vec(2), spring_len, spring_dir(1), spring_dir(2), ...
        F_spring1_2(1), F_spring1_2(2), F1, r1_2(1), r1_2(2), torque1, ...
        P_f2(1), P_f2(2), P_m4(1), P_m4(2), theta_a);
        fprintf('t=%.3f, right',t);
    else
        torque1 = 0;
        warning('No spring torque applied at t=%.4f, x_m=%.4f, spring_len1=%.8f, spring_len2=%.8f, theta_a=%.4f\n', ...
                t, x_m, spring_len1, spring_len2, theta_a);
    end

    fprintf('t=%.2f, e_a=%.4f, int_e_a=%.4f, de_a=%.4f, Kp_a*e_a=%.4f, Ki_a*int=%.4f, Kd_a*de_a=%.4f\n', ...
    t, e_a, integral_e_a, de_a, Kp_a*e_a, Ki_a*integral_e_a, Kd_a * de_a);
    fprintf('t=%.2f, e_t=%.4f, int_e_t=%.4f, de_t=%.4f, Kp_t*e_t=%.4f, Ki_t*int=%.4f, Kd_t*de_t=%.4f\n', ...
    t, e_t, integral_e_t, de_t, Kp_t*e_t, Ki_t*integral_e_t, Kd_t * de_t);

    fprintf('t=%.3f, omega_m=%.4f', t, omega_m);
    fprintf('t=%.3f, x_m=%.4f, e_a=%.4f', t, x_m, e_a);
    fprintf('t=%.3f, S_L=%.8f', t, S_L);
    fprintf('t=%.3f, S_H=%.8f', t, S_H);

    % Heel spring k3 geometry
    P_top = [-d_r-d_d+d_k3; -(h_f-h_k3)];
    local_s3 = [-d_r-d_d+d_k3; -h_a];

    P_bot = rot_a * local_s3 + O1;
    
    % Spring k3 system
    spring_vec3 = P_top - P_bot;
    spring_len3 = norm(spring_vec3);
    
    % Ankle total torque: springs + damping + GRF contributions
    toe_ankle = toe_ankle_ground_reaction_torque(t);
    tau_GRF = ground_reaction_torque(t);
    tau_leg = leg_ground_reaction_torque(t);
    total_torque_a = torque1 - b_a * omega_a + tau_GRF - tau_leg + toe_ankle;
    % total_torque_a = torque1 - b_a * omega_a;

    if spring_len3 >= l_k3
        spring_dir3 = spring_vec3 / spring_len3;
        F_spring3 = k3 * (spring_len3 - l_k3) * spring_dir3; % NOTE: if this seems off, check geometry
        F3 = norm(F_spring3);
        r3 = P_bot - O1;
        torque3 = cross2D(r3, F_spring3);
        total_torque_a = torque1 + torque3 - b_a * omega_a + tau_GRF - tau_leg + toe_ankle;
        % total_torque_a = torque1 + torque3 - b_a * omega_a;
        
        fprintf('t=%.3f, torque3=%.4f, spring_len3=%.4f, F3=%.4f, tau_pid_t=%.4f, omega_t=%.4f\n', ...
        t, torque3, spring_len3, F3, tau_pid_t, omega_t);
    end

    % NOTE: Ankle motor torque is generated via speed loop; not directly added here.

    % Toe joint torque sum
    torque_spring_t = -k2 * (theta_t - 0);
    damping_joint_t = -b_t * omega_t;
    damping_motor_t = - (b_m / n_t^2) * omega_t;
    tau_toe_GRF = toe_ground_reaction_torque(t);
    total_torque_t = tau_pid_t / n_t + torque_spring_t + damping_joint_t + damping_motor_t + tau_toe_GRF;
    
    % Angle limits (with margin)
    theta_a_min = deg2rad(-20);
    theta_a_max = deg2rad(20);
    theta_t_min = deg2rad(-0.5);
    theta_t_max = deg2rad(37.5);
    
    % Hard limit: ankle
    if theta_a < theta_a_min
        y(1) = theta_a_min;
        omega_a = 0;
    elseif theta_a > theta_a_max
        y(1) = theta_a_max;
        omega_a = 0;
    end
    
    % Hard limit: toe
    if theta_t < theta_t_min
        y(5) = theta_t_min;
        omega_t = 0;
    elseif theta_t > theta_t_max
        y(5) = theta_t_max;
        omega_t = 0;
    end

    % State derivatives
    dydt = zeros(8,1);
    dydt(1) = omega_a;
    dydt(2) = total_torque_a / I_a_total;

    dx_m = omega_m * n * p;    % Ball-screw position rate (p = lead per rad)
    fprintf('t=%.3f, dx_m=%.4f', t, dx_m);
    
    % Stroke limit (limit derivatives, not the state)
    x_min = 0; x_max = limit;
    if y(3) <= x_min && dx_m < 0
        dx_m = 0;    
        omega_m = 0;
    elseif y(3) >= x_max && dx_m > 0
        dx_m = 0;
        omega_m = 0;
    end

    dydt(3) = dx_m;
    % dydt(4) = alpha_m;   % Motor angular acceleration (if modeling speed loop dynamics)
    dydt(5) = omega_t;
    dydt(6) = total_torque_t / I_t;
    dydt(7) = e_a;
    dydt(8) = e_t;
    
    % Log to base workspace
    assignin('base', 'spring_len_array', [evalin('base','spring_len_array'); spring_len]);
    assignin('base', 'spring_len3_array', [evalin('base','spring_len3_array'); spring_len3]);
    assignin('base', 'x_m_array', [evalin('base','x_m_array'); x_m]);
    assignin('base', 'omega_m_array', [evalin('base','omega_m_array'); omega_m]);
    assignin('base', 'omega_t_array', [evalin('base','omega_t_array'); omega_t]);
    assignin('base', 'tau_pid_t_array', [evalin('base','tau_pid_t_array'); tau_pid_t]);
end

function theta_ref = ankle_trajectory_real(t)
    % Able-bodied ankle angle trajectory (black solid in your figure)
    % Input: t ∈ [0, 1] (s)
    % Output: theta_ref (rad)

    % Map t to % gait cycle
    gc = t * 100;

    % Data points estimated from figure (deg)
    x_gc =        [0   5  10  15  20  25  30  35  40   45    50   55  60   65    70  75   80   85   90  95 100 105 110 115];
    y_angle_deg = [0  -2 -3.5 -5 -7.5 -5 -2.5  0 1.25 2.5    5   6.5 8.5  10    5   -5  -12.5 -10  -5  -1  0  -2 -3.5 -5];

    % Spline & rad
    y_angle_rad = deg2rad(y_angle_deg);
    theta_ref = interp1(x_gc, y_angle_rad, gc, 'pchip');

    % Out-of-range protection
    theta_ref(gc < 0 | gc > 115) = 0;
end

function theta_ref = toe_trajectory_real(t)
    % First figure’s blue curve (toe)
    % Input: t ∈ [0, 1] (s)
    % Output: theta_ref (rad)

    % Map t → %GC
    gc = t * 100;

    % Data points estimated from figure (deg)
    x_gc =        [0    5    10   15   20  25  30 35   40  45 50   55   60 65  70   75    80  85  90   95  100  105  110 115];
    y_angle_deg = [11 11.25 11.5  12    8   4   2  0  1.75  2  3.5  6    9 15 27.5  37    20  10 10.25 10.5 11 11.25 11.5 12];

    y_angle_rad = deg2rad(y_angle_deg);
    theta_ref = interp1(x_gc, y_angle_rad, gc, 'pchip');

    % Out-of-range protection
    theta_ref(gc < 0 | gc > 115) = 0;
end

function omega_ref = ankle_angular_velocity_real(t)
    dt = 0.001;  % Differencing step (adjust to control rate)
    if t < dt
        % Forward difference near t=0
        theta_now = ankle_trajectory_real(t);
        theta_next = ankle_trajectory_real(t + dt);
        omega_ref = (theta_next - theta_now) / dt;
    else
        % Central difference
        theta_now = ankle_trajectory_real(t);
        theta_prev = ankle_trajectory_real(t - dt);
        omega_ref = (theta_now - theta_prev) / dt;
    end
end

function omega_ref = toe_angular_velocity_real(t)
    dt = 0.001;  % Differencing step (adjust to control rate)
    if t < dt
        % Forward difference near t=0
        theta_now = toe_trajectory_real(t);
        theta_next = toe_trajectory_real(t + dt);
        omega_ref = (theta_next - theta_now) / dt;
    else
        % Central difference
        theta_now = toe_trajectory_real(t);
        theta_prev = toe_trajectory_real(t - dt);
        omega_ref = (theta_now - theta_prev) / dt;
    end
end

function z = cross2D(a, b)
    % 2D scalar “z-component” of cross: z = a_x*b_y - a_y*b_x
    z = a(1)*b(2) - a(2)*b(1);
end

function visualize_motion_v7(t, y)
    % Geometry/coordinate system:
    % Origin is at the interface between the receiving plane’s top surface and the footplate’s top surface.
    % Ignore receiving-plane protrusion height and footplate thickness (actual footplate thickness 1.4 cm).
    % X rightward, Y upward. Receiving plane extends d_r to left/right.
    d_r = 5.6/100;
    d_d = 1.55/100; % Distance from heel to left end of receiving plane
    l_k = 23/100;   % Footplate length (no toe)
    l_t = 7.5/100;  % Toe length
    h_a = 7/100;    % Ankle height (to sole)
    h_m = 3/100;    % Transmission pivot height (to sole)
    h_t = 1.3/100;  % Toe joint height (to sole)
    h_f = 18.8/100; % Total foot height (to sole)
    d_a = 7.15/100; % Horizontal ankle position from heel
    h_s = 11.8/100; % Vertical ankle-to-receiving-plane distance
    l_k3 = 4/100;   % Heel spring k3 free length (compression)
    h_k3 = 4/100;   % Vertical distance from k3 top to footplate
    d_k3 = 4.5/100; % Horizontal distance from k3 to heel
    h_c = 0/100;    % Vertical offset between transmission pivot and coupling (0 in new design)
    l_c = 0/100;    % Equivalent coupling length (ignored)

    % Design-dependent
    d_k1_1 = 2.5/100;  % Spring k1 plantarflexion – footplate left end position (from ankle)
    l_k1_1 = 4/100;    % Spring k1 plantarflexion – free length
    d_k1_2 = 10.4/100; % Spring k1 dorsiflexion – footplate left end position (from ankle)
    l_k1_2 = 3.2/100;  % Spring k1 dorsiflexion – free length
    t_k1 = h_m;        % Vertical offset of k1 (aligned with screw & coupling)

    l_g1 = 3.2/100;  % Ball screw fixed horizontal length (left)
    l_g2 = 1.225/100;% Ball screw nut seat extra horizontal (middle)
    l_g3 = 0.7/100;  % Ball screw nut seat thickness (right)

    for i = 1:10:length(t)
        theta_a = y(i,1);
        x_m = y(i,3);
        theta_t = y(i,5);

        clf; hold on;
        
        % Rod above the ankle joint
        plot([0, 0], [0, -h_s], 'k-', 'LineWidth', 3);

        % Ankle joint
        O1 = [0; -h_s]; % Ankle coordinates
        plot(O1(1), O1(2), 'ko', 'MarkerFaceColor', 'w'); 
        draw_circle(O1, 0.01, 'k', 1);
        draw_circle(O1, 0.015, 'k', 1);
        
        % GRF receiving plane
        plot([-d_r, d_r], [0, 0], 'b-', 'LineWidth', 3);

        % Footplate
        foot_pts = [-d_r-d_d, -h_f; -d_r-d_d+l_k, -h_f]';
        rot_a = [cos(theta_a), -sin(theta_a); sin(theta_a), cos(theta_a)];
        foot_rot = rot_a * (foot_pts - O1) + O1;
        plot(foot_rot(1,:), foot_rot(2,:), 'b-', 'LineWidth', 3);

        % Left endpoint of footplate (initial)
        P0 = [-d_r-d_d; -h_f];

        % Spring 1 – plantarflexion
        P1 = [-d_r-d_d+l_k; -h_f];
        dir = (P1 - P0) / norm(P1 - P0);
        P_local1_1 = P0 + dir * (d_k1_1+d_a);
        P_global1_1 = rot_a * (P_local1_1 - O1) + O1;

        % Spring 1 plantarflexion – left endpoint (fixed on footplate)
        P_f1_x = P_global1_1(1) - t_k1*sin(theta_a);
        P_f1_y = P_global1_1(2) + t_k1*cos(theta_a);
        P_f1 = [P_f1_x; P_f1_y];

        % Link from footplate anchor to offset endpoint
        plot([P_global1_1(1), P_f1(1)], [P_global1_1(2), P_f1(2)], 'k-', 'LineWidth', 3);
        
        % Spring 1 – dorsiflexion
        P1 = [-d_r-d_d+l_k; -h_f];
        dir = (P1 - P0) / norm(P1 - P0);
        P_local1_2 = P0 + dir * (d_k1_2+d_a);
        P_global1_2 = rot_a * (P_local1_2 - O1) + O1;

        % Spring 1 dorsiflexion – right endpoint (fixed on footplate)
        P_f2_x = P_global1_2(1) - t_k1*sin(theta_a);
        P_f2_y = P_global1_2(2) + t_k1*cos(theta_a);
        P_f2 = [P_f2_x; P_f2_y];

        % Link from footplate anchor to offset endpoint
        plot([P_global1_2(1), P_f2(1)], [P_global1_2(2), P_f2(2)], 'k-', 'LineWidth', 3);
        
        % Ankle rod to footplate
        P_local2 = P0 + dir * d_a;
        P_global2 = rot_a * (P_local2 - O1) + O1;
        plot([P_global2(1), 0], [P_global2(2), -h_s], 'k-', 'LineWidth', 3);
        
        % Transmission pivot
        P_t = [0.00;-(h_f-h_m)];
        plot(0.00, -(h_f-h_m), 'ko', 'MarkerFaceColor', 'w');
        
        % Ball-screw segments
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

        % Nut-seat offsets to footplate
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

        % Heel spring
        P_top = [-d_r-d_d+d_k3; -(h_f-h_k3)];
        local_s3 = [-d_r-d_d+d_k3; -h_a];
        P_bot = rot_a * local_s3 + O1;
        plot([P_top(1), P_bot(1)], [P_top(2), P_bot(2)], 'r--', 'LineWidth', 2);
        
        % Upper rod of k3
        plot([P_top(1), P_top(1)], [P_top(2), 0], 'k-', 'LineWidth', 3);
        
        plot(P_bot(1), P_bot(2), 'ko', 'MarkerFaceColor', 'm');
        plot(P_top(1), P_top(2), 'ko', 'MarkerFaceColor', 'm');
        
        % Toe joint
        O2_local = [-d_r-d_d+l_k; -h_f+h_t];
        O2_local2 = [-d_r-d_d+l_k; -h_f];
        O2 = rot_a * (O2_local - O1) + O1;
        O2_2 = rot_a * (O2_local2 - O1) + O1;
        draw_circle(O2, 0.01, 'k', 1);

        toe_length = l_t;
        rot_t = [cos(theta_t), -sin(theta_t); sin(theta_t), cos(theta_t)];
        toe_vec = rot_t * [toe_length; 0];
        toe_end = O2_2 + toe_vec;
        
        plot([O2_2(1), O2(1)], [O2_2(2), O2(2)], 'k-', 'LineWidth', 3);
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

% Torque on the sole (acting on the ankle) during 0–50% GC; heel friction torque ignored
function tau_GRF = ground_reaction_torque(t)
    % t: seconds (scalar or vector)

    % Simulation parameters
    feet_length = 0.25;      % Foot length (m)
    Body_weight = 60;        % Body weight (kg)
    T_stance = 0.5;          % Stance duration (s)

    % r_x data (% foot length)
    r_x = [10 15 22.5 27.5 30 32.5 37.5 42 45 51 56 60 65 70 72.5 74.5 76];
    x_old = linspace(0.15, T_stance, length(r_x));   % Map across 0–0.5 s

    % Interpolation grid
    t_interp = linspace(0, T_stance, 10);
    r_x_interp = interp1(x_old, r_x, t_interp, 'linear');
    r_x_net = r_x_interp - 25;                    % Ankle at 25%
    r_x_actual = r_x_net * feet_length / 100;     % m

    % Vertical GRF (BW units → N)
    F_y = [0 9 12 11.5 10.5 9.5 9 9.5 10.5 12 11];
    F_y_interp = interp1(linspace(0, T_stance, length(F_y)), F_y, t_interp);
    F_y_actual = F_y_interp * Body_weight;

    % Torque for 0–0.5 s
    tau_half = r_x_actual .* F_y_actual;

    % Output at generic t
    tau_GRF = zeros(size(t));
    idx = t <= T_stance;
    tau_GRF(idx) = interp1(t_interp, tau_half, t(idx), 'linear', 0);
end

% Toe torque during ~50%–60% GC
function tau_toe_GRF = toe_ground_reaction_torque(t)
    % t: seconds (scalar or vector)

    % Simulation parameters
    feet_length = 0.25;      % Foot length (m)
    Body_weight = 60;        % Body weight (kg)
    T_start = 0.65;          % Toe-support start time
    T_stance = 0.1;          % Toe-support duration (s)

    % Interpolation grid (toe support period)
    t_interp = linspace(T_start, T_start + T_stance, 10);

    % r_x data (% foot length)
    r_x = [79 83 91];        % COP moves forward on toe
    t_r = linspace(T_start, T_start + T_stance, length(r_x));
    r_x_interp = interp1(t_r, r_x, t_interp, 'linear');

    % Relative to ankle at 75% FL here
    r_x_net = r_x_interp - 75;
    r_x_actual = r_x_net * feet_length / 100;  % m

    % Vertical GRF (BW units → N)
    F_y = [9 5 1];
    t_f = linspace(T_start, T_start + T_stance, length(F_y));
    F_y_interp = interp1(t_f, F_y, t_interp, 'linear');
    F_y_actual = F_y_interp * Body_weight;

    % Torque
    tau_toe_half = r_x_actual .* F_y_actual;

    % Output
    tau_toe_GRF = zeros(size(t));
    idx = (t >= T_start) & (t <= T_start + T_stance);
    tau_toe_GRF(idx) = interp1(t_interp, tau_toe_half, t(idx), 'linear', 0);
end

% Estimated leg torque acting on ankle across GC
function tau_leg_GRF = leg_ground_reaction_torque(t)
    % t: seconds (scalar or vector)

    Body_weight = 60;   % kg
    T_stance = 1;       % s

    % Interp grid
    t_interp = linspace(0.15, T_stance, 20);

    % r_x (m) and F_y (BW) series
    r_x = [-5  -2.5   0    1    2    4   6   8   10   12  13 14  15 0 0 0 0 0 0 0 0]/100;
    F_y = [0     9    12  11.5 10.5 9.5 9  9.5 10.5 12  11  6   1  0 0 0 0 0 0 0 0]; % 21 points

    r_x_interp = interp1(linspace(0, T_stance, length(r_x)), r_x, t_interp);
    F_y_interp = interp1(linspace(0, T_stance, length(F_y)), F_y, t_interp);
    F_y_actual = F_y_interp * Body_weight;

    tau_leg_half = r_x_interp .* F_y_actual;

    tau_leg_GRF = zeros(size(t));
    idx = t <= T_stance;
    tau_leg_GRF(idx) = interp1(t_interp, tau_leg_half, t(idx), 'linear', 0);
end

% Toe push-off (≈50%–60% GC): reaction torque of toe on forefoot
function toe_ankle = toe_ankle_ground_reaction_torque(t)
    % t: seconds (scalar or vector)

    % Simulation parameters
    T_start = 0.65; 
    T_stance = 0.1;             

    % Time nodes corresponding to M_toe_ankle samples
    t_interp = linspace(0, T_stance, length([70 40 20]));
    M_toe_ankle = [70 40 20];   % Nm at sample points

    toe_ankle = zeros(size(t)); % Initialize
    
    % Indices during toe-support interval
    idx = (t >= T_start) & (t <= T_start + T_stance);
    
    % Localize time
    t_local = t(idx) - T_start;
    
    % Interpolate (0 outside range)
    toe_ankle(idx) = interp1(t_interp, M_toe_ankle, t_local, 'linear', 0);
end

function tau = pyStyleCross2D(a, b)
    % Input: a = [x1,y1], b = [x2,y2]
    % Output: np.cross([a,0],[b,0])[2]
    c3 = cross([a(1), a(2), 0], [b(1), b(2), 0]);
    tau = c3(3);
end

% % Example usage:
% % Time vector (0–1 s, 100 points)
% t = linspace(0, 1, 100);
% % Net torque example
% tau = ground_reaction_torque(t) - leg_ground_reaction_torque(t) + toe_ankle_ground_reaction_torque(t);
% % Plot
% figure;
% plot(t, tau, 'LineWidth', 2);
% xlabel('Time (s)');
% ylabel('Leg Ground Reaction Torque (Nm)');
% title('Ground Reaction Torque vs Time');
% grid on;
