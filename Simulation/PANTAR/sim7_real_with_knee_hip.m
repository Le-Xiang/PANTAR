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
    
    % Parameters
    % Rough mass/inertia estimates: forefoot/ankle assembly dominates; toe section light.
    I_a = 0.01;      % Footplate MOI about ankle (kg·m^2)
    I_t = 0.0001;    % Toe joint MOI (kg·m^2)
    m_t = 0.1;       % Toe mass (kg)
    d = 0.155;       % Distance toe joint to ankle (m)
    eta = 0.9;       % Efficiency factor η 
    
    % Parallel-axis contribution from the toe about the ankle
    I_toe_at_ankle = I_t + m_t * d^2;
    I_a_total = I_a + I_toe_at_ankle;
    % If the toe does not rotate with ankle in a specific phase, set I_a_total = I_a.
    
    % System 1 – Ankle SEA/spring pair (plantarflexion / dorsiflexion sides)
    k1_1 = 156920; % Spring k1 (plantarflexion side), N/m equivalent to torque via geometry
    k1_2 = 156920; % Spring k1 (dorsiflexion side)
    n = 1;         % Gear ratio for k1 system
    p_m_per_rev = 0.01; % Ball-screw lead (m/rev). Using 10 mm here.
    p = p_m_per_rev / (2 * pi);  % m/rad motor → nut travel per rad
    theta_a0 = deg2rad(-5);      % Initial ankle angle
    
    % System 3 – Heel spring
    k3 = 2380; % Spring k3 (heel)
    
    % System 2 – Toe joint
    n_t = 0.5;                 % Toe gear ratio
    theta_t0 = deg2rad(12);    % Initial toe angle
    I_actual = 0;              % Measured motor current (placeholder external input)
    k2 = 1.075;                % Toe torsional spring (N·m/rad)
    
    % Damping
    b_a = 0.05;   % Ankle damping N·m·s/rad
    b_t = 0.01;   % Toe joint damping N·m·s/rad
    b_m = 0.005;  % Motor-axis damping (toe) N·m·s/rad

    % Initial x_m based on initial ankle angle
    % x_m = 2.075/100; % for 0 deg
    x_m = 1.4/100; % for -5 deg

    % State vector:
    % y = [theta_a; omega_a; x_m; (omega_m unused); theta_t; omega_t; int_e_a; int_e_t]
    y0 = [theta_a0; 0; x_m; 0; theta_t0; 0; 0; 0]; 

    tspan = [0 1.1];
    % options = odeset('RelTol', 1e-8, 'AbsTol', 1e-8);

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
    title('Ankle Angle Tracking (solid: actual, dashed: reference)');
    legend('Actual', 'Reference');
    grid on;
    
    % Toe tracking
    h3 = figure();
    plot(t, y(:,5)*180/pi, 'LineWidth', 2); hold on;
    plot(t, arrayfun(@toe_trajectory_real, t)*180/pi, '--r', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    title('Toe Angle Tracking (solid: actual, dashed: reference)');
    legend('Actual', 'Reference');
    grid on;
   
    % Stats collection
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
    fprintf('min tau_{pid,t} = %.4f Nm\n', min(abs(tau_pid_t_array)));
    fprintf('max tau_{pid,t} = %.4f Nm\n', max(abs(tau_pid_t_array)));
    Q3 = prctile(abs(tau_pid_t_array), 97.5);
    fprintf('Q3 (97.5%%) = %.4f Nm\n', Q3);

    % Tracking errors
    theta1_actual = y(:,1);                   % ankle
    theta2_actual = y(:,5);                   % toe
    theta1_desired = arrayfun(@ankle_trajectory_real, t);
    theta2_desired = arrayfun(@toe_trajectory_real, t);
    ankle_error = abs(theta1_actual - theta1_desired);
    toe_error   = abs(theta2_actual - theta2_desired);
    ankle_error_sum = sum(ankle_error);
    toe_error_sum   = sum(toe_error);
    fprintf('Cumulative ankle error: %.4f rad\n', ankle_error_sum);
    fprintf('Cumulative toe error: %.4f rad\n', toe_error_sum);

    % h4 = figure();
    % plot(t, ankle_error, 'r', 'LineWidth', 2); hold on;
    % plot(t, toe_error, 'b', 'LineWidth', 2);
    % xlabel('Time (s)');
    % ylabel('Error (rad)');
    % legend('Ankle error', 'Toe error');
    % title('Angle Tracking Errors vs Time');
    % grid on;
end

function dydt = dynamics_v7(t, y, I_a_total, I_t, ...
    k1_1,k1_2, n, p, k3, n_t, b_a, b_t, b_m, eta, I_actual, k2)

    theta_a = y(1); % Ankle angle
    omega_a = y(2); % Ankle angular velocity
    x_m     = y(3); % Ball-screw nut position
    % omega_m = y(4); % (unused state slot)
    theta_t = y(5); % Toe angle
    omega_t = y(6); % Toe angular velocity
    integral_e_a = y(7); 
    integral_e_t = y(8);

    % References
    theta_a_ref = ankle_trajectory_real(t);
    theta_t_ref = toe_trajectory_real(t);
    omega_a_ref = ankle_angular_velocity_real(t);
    omega_t_ref = toe_angular_velocity_real(t);

    % --- Ankle PID → target motor speed ---
    e_a  = theta_a_ref - theta_a;
    de_a = omega_a_ref - omega_a;  % velocity error as derivative proxy
    Kp_a = 600; Ki_a = 1000; Kd_a = 6000;
    omega_m = (Kp_a*e_a + Ki_a*integral_e_a + Kd_a * de_a);

    % --- Toe PID (torque control) ---
    e_t  = theta_t_ref - theta_t;
    de_t = omega_t_ref - omega_t;  
    Kp_t = 800; Ki_t = 10; Kd_t = 10; % Kd_t currently not used below
    % tau_pid_t = Kp_t*e_t + Ki_t*integral_e_t + Kd_t*de_t;
    tau_pid_t = Kp_t*e_t + Ki_t*integral_e_t;

    % Torque → current reference (placeholder mapping)
    k_t = 0.5;
    I_ref = tau_pid_t / k_t;
    % error_I = I_ref - I_actual; % PI current loop could be added

    % Geometry (meters)
    d_r = 5.6/100;
    d_d = 1.55/100; % Heel to left edge of GRF plane
    l_k = 23/100;   % Footplate length (no toe)
    l_t = 7.5/100;  % Toe length
    h_a = 7/100;    % Ankle height above sole
    h_m = 3/100;    % Transmission pivot height above sole
    h_t = 1.3/100;  % Toe joint height above sole
    h_f = 18.8/100; % Overall shank height reference
    d_a = 7.15/100; % Ankle horizontal position from heel
    h_s = 11.8/100; % Ankle vertical distance from GRF plane
    l_k3 = 4/100;   % Spring k3 nominal length (compressive spring)
    h_k3 = 4/100;   % Spring k3 top vertical offset
    d_k3 = 4.5/100; % Spring k3 horizontal from heel
    h_c = 0/100;    % Transmission pivot to coupler vertical offset
    l_c = 0/100;    % Coupler equivalent length
    limit = 3.75/100; % Screw stroke

    % Spring k1 geometry (to be matched to design)
    d_k1_1 = 2.5/100;  % PF side fixed point offset from ankle along foot
    l_k1_1 = 4/100;    % PF spring nominal length
    d_k1_2 = 10.4/100; % DF side fixed point offset
    l_k1_2 = 3.2/100;  % DF spring nominal length
    t_k1 = h_m;        % Vertical offset so spring endpoints align with screw/coupler axis

    % Screw assembly piecewise lengths
    l_g1 = 3.2/100; 
    l_g2 = 1.225/100;
    l_g3 = 0.7/100;
   
    % Ankle joint origin on GRF frame
    O1 = [0; -h_s];

    % Footplate endpoints before ankle rotation
    foot_pts = [-d_r-d_d, -h_f; -d_r-d_d+l_k, -h_f]';
    rot_a = [cos(theta_a), -sin(theta_a); sin(theta_a), cos(theta_a)];
    foot_rot = rot_a * (foot_pts - O1) + O1;

    % Leftmost foot endpoint
    P0 = [-d_r-d_d; -h_f];

    % PF spring fixed point on foot
    P1 = [-d_r-d_d+l_k; -h_f];
    dir = (P1 - P0) / norm(P1 - P0);
    P_local1_1 = P0 + dir * (d_k1_1+d_a);
    P_global1_1 = rot_a * (P_local1_1 - O1) + O1;

    % PF side foot-fixed end
    P_f1_x = P_global1_1(1) - t_k1*sin(theta_a);
    P_f1_y = P_global1_1(2) + t_k1*cos(theta_a);
    P_f1 = [P_f1_x; P_f1_y];
    
    % DF spring fixed point on foot
    P1 = [-d_r-d_d+l_k; -h_f];
    dir = (P1 - P0) / norm(P1 - P0);
    P_local1_2 = P0 + dir * (d_k1_2+d_a);
    P_global1_2 = rot_a * (P_local1_2 - O1) + O1;

    % DF side foot-fixed end
    P_f2_x = P_global1_2(1) - t_k1*sin(theta_a);
    P_f2_y = P_global1_2(2) + t_k1*cos(theta_a);
    P_f2 = [P_f2_x; P_f2_y];

    % Ankle link point on foot
    P_local2 = P0 + dir * d_a;
    P_global2 = rot_a * (P_local2 - O1) + O1;

    % Transmission pivot
    P_t = [0.00;-(h_f-h_m)];
    
    % Screw endpoints
    % Left fixed part
    P_m_x = P_t(1) + l_g1*cos(theta_a);
    P_m_y = P_t(2) + l_g1*sin(theta_a);
    P_m1 = [P_m_x;P_m_y];
    
    % Nut left side
    P_m_x = P_t(1) + (x_m+l_g1)*cos(theta_a);
    P_m_y = P_t(2) + (x_m+l_g1)*sin(theta_a);
    P_m2 = [P_m_x;P_m_y];
    
    % Nut sleeve left end (PF spring right end)
    P_m_x = P_t(1) + (x_m+l_g1+l_g2)*cos(theta_a);
    P_m_y = P_t(2) + (x_m+l_g1+l_g2)*sin(theta_a);
    P_m3 = [P_m_x;P_m_y];

    % Nut sleeve right end (DF spring left end)
    P_m_x = P_t(1) + (x_m+l_g1+l_g2+l_g3)*cos(theta_a);
    P_m_y = P_t(2) + (x_m+l_g1+l_g2+l_g3)*sin(theta_a);
    P_m4 = [P_m_x;P_m_y];
 
    % Sleeve points projected to footframe offsets
    P_f3_x = P_m3(1) + t_k1*sin(theta_a);
    P_f3_y = P_m3(2) - t_k1*cos(theta_a);
    P_f3 = [P_f3_x; P_f3_y];

    P_f4_x = P_m4(1) + t_k1*sin(theta_a);
    P_f4_y = P_m4(2) - t_k1*cos(theta_a);
    P_f4 = [P_f4_x; P_f4_y];
    
    spring_vec1 = P_m3 - P_f1;
    spring_len1 = norm(spring_vec1);
    spring_vec2 = P_m4 - P_f2;
    spring_len2 = norm(spring_vec2);

    S_L = norm(P_f1 - P_f2);
    S_H = norm(P_m3 - P_m4);

    if spring_len1 <= (4.01/100)
        % k1 plantarflexion side (left spring)
        spring_vec = P_m3 - P_f1;
        spring_len = norm(spring_vec);
        spring_dir = spring_vec / spring_len;
        F_spring1_1 = k1_1 * (l_k1_1 - spring_len) * spring_dir;
        F1 = norm(F_spring1_1);
        r1_1 = P_f1 - O1; 
        % Sign note: torque from 2-D cross product may need a sign flip depending on convention.
        torque1 = -cross2D(r1_1, F_spring1_1);

        fprintf(['\n--- Spring Debug (PF) @ t=%.3f ---\n', ...
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
        fprintf('t=%.3f, left\n',t);
    
    elseif spring_len2 <= (3.21/100)
        % k1 dorsiflexion side (right spring)
        spring_vec = P_m4 - P_f2;
        spring_len = norm(spring_vec);
        spring_dir = spring_vec / spring_len;
        F_spring1_2 = k1_2 * (l_k1_2 - spring_len) * spring_dir;
        F1 = norm(F_spring1_2);
        r1_2 = P_f2 - O1; 
        torque1 = -cross2D(r1_2, F_spring1_2);

        fprintf(['\n--- Spring Debug (DF) @ t=%.3f ---\n', ...
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
        fprintf('t=%.3f, right\n',t);
    else
        torque1 = 0;
        warning('No spring torque at t=%.4f, x_m=%.4f, spring_len1=%.8f, spring_len2=%.8f, theta_a=%.4f\n', ...
            t, x_m, spring_len1, spring_len2, theta_a);
    end

    fprintf('t=%.2f, e_a=%.4f, int_e_a=%.4f, de_a=%.4f, Kp*e=%.4f, Ki*int=%.4f, Kd*de=%.4f\n', ...
        t, e_a, integral_e_a, de_a, Kp_a*e_a, Ki_a*integral_e_a, Kd_a*de_a);
    fprintf('t=%.2f, e_t=%.4f, int_e_t=%.4f, de_t=%.4f, Kp*e=%.4f, Ki*int=%.4f, Kd*de=%.4f\n', ...
        t, e_t, integral_e_t, de_t, Kp_t*e_t, Ki_t*integral_e_t, Kd_t*de_t);

    fprintf('t=%.3f, omega_m=%.4f\n', t, omega_m);
    fprintf('t=%.3f, x_m=%.4f, e_a=%.4f\n', t, x_m, e_a);
    fprintf('t=%.3f, S_L=%.8f\n', t, S_L);
    fprintf('t=%.3f, S_H=%.8f\n', t, S_H);

    % Heel spring endpoints
    P_top = [-d_r-d_d+d_k3; -(h_f-h_k3)];
    local_s3 = [-d_r-d_d+d_k3; -h_a];
    P_bot = rot_a * local_s3 + O1;
    
    % Spring k3
    spring_vec3 = P_top - P_bot;
    spring_len3 = norm(spring_vec3);
    
    % External torques
    toe_ankle = toe_ankle_ground_reaction_torque(t);
    tau_GRF   = ground_reaction_torque(t);
    tau_leg   = leg_ground_reaction_torque(t);
    total_torque_a = torque1 - b_a * omega_a + tau_GRF - tau_leg + toe_ankle;
    % total_torque_a = torque1 - b_a * omega_a;

    if spring_len3 >= l_k3
        spring_dir3 = spring_vec3 / spring_len3;
        F_spring3 = k3 * (spring_len3 - l_k3) * spring_dir3;
        F3 = norm(F_spring3);
        r3 = P_bot - O1;
        torque3 = cross2D(r3, F_spring3);
        total_torque_a = torque1 + torque3 - b_a * omega_a + tau_GRF - tau_leg + toe_ankle;
        % total_torque_a = torque1 + torque3 - b_a * omega_a;
        
        fprintf('t=%.3f, torque3=%.4f, spring_len3=%.4f, F3=%.4f, tau_pid_t=%.4f, omega_t=%.4f\n', ...
            t, torque3, spring_len3, F3, tau_pid_t, omega_t);
    end

    % Toe dynamics contributions
    torque_spring_t = -k2 * (theta_t - 0);
    damping_joint_t = -b_t * omega_t;
    damping_motor_t = -(b_m / n_t^2) * omega_t;
    tau_toe_GRF     = toe_ground_reaction_torque(t);
    total_torque_t  = (tau_pid_t / n_t) + torque_spring_t + damping_joint_t + damping_motor_t + tau_toe_GRF;
    
    % Joint limits (rad)
    theta_a_min = deg2rad(-20);
    theta_a_max = deg2rad(20);
    theta_t_min = deg2rad(-0.5);
    theta_t_max = deg2rad(37.5);
    
    % Hard limits
    if theta_a < theta_a_min
        y(1) = theta_a_min;
        omega_a = 0;
    elseif theta_a > theta_a_max
        y(1) = theta_a_max;
        omega_a = 0;
    end
    
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

    dx_m = omega_m * n * p;    % Screw nut speed
    fprintf('t=%.3f, dx_m=%.4f\n', t, dx_m);
    
    % Stroke limits (limit the derivative rather than state)
    x_min = 0; x_max = limit;
    if y(3) <= x_min && dx_m < 0
        dx_m = 0;
        omega_m = 0;
    elseif y(3) >= x_max && dx_m > 0
        dx_m = 0;
        omega_m = 0;
    end

    dydt(3) = dx_m;
    % dydt(4) = alpha_m; % not used
    dydt(5) = omega_t;
    dydt(6) = total_torque_t / I_t;
    dydt(7) = e_a;
    dydt(8) = e_t;
    
    % Log arrays to base workspace
    assignin('base', 'spring_len_array', [evalin('base','spring_len_array'); spring_len]);
    assignin('base', 'spring_len3_array', [evalin('base','spring_len3_array'); spring_len3]);
    assignin('base', 'x_m_array', [evalin('base','x_m_array'); x_m]);
    assignin('base', 'omega_m_array', [evalin('base','omega_m_array'); omega_m]);
    assignin('base', 'omega_t_array', [evalin('base','omega_t_array'); omega_t]);
    assignin('base', 'tau_pid_t_array', [evalin('base','tau_pid_t_array'); tau_pid_t]);
end

function theta_ref = ankle_trajectory_real(t)
    % Able-bodied ankle angle reference (rad) vs time normalized to 0–1 s
    gc = t * 100; % % gait cycle
    x_gc =        [0   5  10  15  20  25  30  35  40   45    50   55  60   65    70  75    80   85   90  95 100 105 110];
    y_angle_deg = [-5 -7.5 -5 -2.5  0 1.25 2.5   5   6.5  8.5  10    5   -5  -12.5 -10  -5   -1    0   -2 -3.5 -5 -5 -5];
    y_angle_rad = deg2rad(y_angle_deg);
    theta_ref = interp1(x_gc, y_angle_rad, gc, 'pchip');
    theta_ref(gc < 0 | gc > 110) = deg2rad(-5);
end

function theta_ref = toe_trajectory_real(t)
    % Able-bodied toe angle reference (rad)
    gc = t * 100;
    x_gc =        [0  5 10 15 20 25 30 35 40 45 50 55   60 65 70 75    80   85 90  95 100 105 110];
    y_angle_deg = [12 8  4  2  0 1.75 2 3.5  6  9 15 27.5 37 20 10 10.25 10.5 11 11.25 11.5 12 12 12];
    y_angle_rad = deg2rad(y_angle_deg);
    theta_ref = interp1(x_gc, y_angle_rad, gc, 'pchip');
    theta_ref(gc < 0 | gc > 110) = deg2rad(12);
end

function theta_ref = knee_trajectory_real(t)
    % Knee angle reference (rad)
    gc = t * 100;
    x_gc =        [0   5  10  15  20  25  30  35  40   45    50   55  60   65    70  75    80   85   90  95 100 105 110];
    y_angle_deg = [6   4  12  13  12  10   8   6   5    6    10   20  31   45    58  55    45   30   20  10  6   6   6];
    y_angle_rad = deg2rad(y_angle_deg);
    theta_ref = interp1(x_gc, y_angle_rad, gc, 'pchip');
    theta_ref(gc < 0 | gc > 110) = deg2rad(6);
end

function theta_ref = hip_trajectory_real(t)
    % Hip angle reference (rad)
    gc = t * 100;
    x_gc =        [0   5  10  15  20  25  30  35  40   45    50   55  60   65    70  75    80   85   90  95 100 105 110];
    y_angle_deg = [11  9   8   4   0   -5  -8  -10 -13 -14  -15  -16 -10   -5    2.5 7.5   10   11   12  12  11  11 11];
    y_angle_rad = deg2rad(y_angle_deg);
    theta_ref = interp1(x_gc, y_angle_rad, gc, 'pchip');
    theta_ref(gc < 0 | gc > 110) = deg2rad(11);
end

function omega_ref = ankle_angular_velocity_real(t)
    % Numerical derivative of ankle reference
    dt = 0.001;
    if t < dt
        theta_now  = ankle_trajectory_real(t);
        theta_next = ankle_trajectory_real(t + dt);
        omega_ref = (theta_next - theta_now) / dt;
    else
        theta_now  = ankle_trajectory_real(t);
        theta_prev = ankle_trajectory_real(t - dt);
        omega_ref = (theta_now - theta_prev) / dt;
    end
end

function omega_ref = toe_angular_velocity_real(t)
    % Numerical derivative of toe reference
    dt = 0.001;
    if t < dt
        theta_now  = toe_trajectory_real(t);
        theta_next = toe_trajectory_real(t + dt);
        omega_ref = (theta_next - theta_now) / dt;
    else
        theta_now  = toe_trajectory_real(t);
        theta_prev = toe_trajectory_real(t - dt);
        omega_ref = (theta_now - theta_prev) / dt;
    end
end

function z = cross2D(a, b)
    % 2-D scalar cross product (z-component of 3-D cross)
    z = a(1)*b(2) - a(2)*b(1);
end

function visualize_motion_v7(t, y)
    % Geometry (all in meters). Global frame originally at GRF plane center.
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
    l_k3 = 4/100; 
    h_k3 = 4/100; 
    d_k3 = 4.5/100; 
    h_c = 0/100; 
    l_c = 0/100; 

    % Spring k1 geometry
    d_k1_1 = 2.5/100; 
    l_k1_1 = 4/100; 
    d_k1_2 = 10.4/100; 
    l_k1_2 = 3.2/100; 
    t_k1 = h_m; 

    l_g1 = 3.2/100;
    l_g2 = 1.225/100; 
    l_g3 = 0.7/100; 

    % Thigh/hip placeholders for visualization
    d_f_k = 30/100; 
    d_k_h = 42.5/100; 

    for i = 1:10:length(t)
        theta_a = y(i,1);
        x_m     = y(i,3);
        theta_t = y(i,5);

        clf; hold on;

        theta_h = hip_trajectory_real(t(i));
        theta_k = knee_trajectory_real(t(i));

        % New frame with hip as origin
        O_h = [0; 0];
        O_k = [d_k_h * sin(theta_h); -d_k_h * cos(theta_h)];
        O_GRF = O_k + [-d_f_k * sin(theta_k-theta_h); -d_f_k * cos(theta_k-theta_h)];

        plot(O_h(1), O_h(2), 'ko', 'MarkerFaceColor', 'w'); 
        draw_circle(O_h, 0.01, 'k', 1);
        draw_circle(O_h, 0.015, 'k', 1);

        plot(O_k(1), O_k(2), 'ko', 'MarkerFaceColor', 'w'); 
        draw_circle(O_k, 0.01, 'k', 1);
        draw_circle(O_k, 0.015, 'k', 1);

        plot([0, O_k(1)], [0, O_k(2)], 'k-', 'LineWidth', 3);
        plot([O_k(1), O_GRF(1)], [O_k(2), O_GRF(2)], 'k-', 'LineWidth', 3);

        % GRF plane segment
        plot([O_GRF(1)-d_r * cos(theta_k-theta_h), O_GRF(1)+d_r * cos(theta_k-theta_h)], ...
             [O_GRF(2)+d_r * sin(theta_k-theta_h), O_GRF(2)-d_r * sin(theta_k-theta_h)], ...
             'b-', 'LineWidth', 3);

        % Ankle joint
        O1 = [O_GRF(1)-h_s * sin(theta_k-theta_h);
              O_GRF(2)-h_s * cos(theta_k-theta_h)];
        plot([O_GRF(1), O1(1)], [O_GRF(2), O1(2)], 'k-', 'LineWidth', 3);
        plot(O1(1), O1(2), 'ko', 'MarkerFaceColor', 'w'); 
        draw_circle(O1, 0.01, 'k', 1);
        draw_circle(O1, 0.015, 'k', 1);

        % Footplate line
        foot_pts = [ ...
            O_GRF(1)-h_f * sin(theta_k-theta_h)+ (-d_r-d_d)     * cos(theta_k-theta_h),   O_GRF(1)-h_f * sin(theta_k-theta_h)+ (-d_r-d_d+l_k) * cos(theta_k-theta_h);
            O_GRF(2)-h_f * cos(theta_k-theta_h)- (-d_r-d_d)     * sin(theta_k-theta_h),   O_GRF(2)-h_f * cos(theta_k-theta_h)- (-d_r-d_d+l_k) * sin(theta_k-theta_h)
        ];
        rot_a = [cos(theta_a), -sin(theta_a); sin(theta_a), cos(theta_a)];
        O1_expanded = repmat(O1, 1, size(foot_pts,2));
        foot_rot = rot_a * (foot_pts - O1_expanded) + O1_expanded;
        plot(foot_rot(1,:), foot_rot(2,:), 'b-', 'LineWidth', 3);

        % PF spring foot-fixed point
        P0 = [O_GRF(1)-h_f * sin(theta_k-theta_h)+(-d_r-d_d) * cos(theta_k-theta_h);
              O_GRF(2)-h_f * cos(theta_k-theta_h)-(-d_r-d_d) * sin(theta_k-theta_h)];
        P1 = [O_GRF(1)-h_f * sin(theta_k-theta_h)+(-d_r-d_d+l_k) * cos(theta_k-theta_h);
              O_GRF(2)-h_f * cos(theta_k-theta_h)-(-d_r-d_d+l_k) * sin(theta_k-theta_h)];
        dir = (P1 - P0) / norm(P1 - P0);
        P_local1_1 = P0 + dir * (d_k1_1+d_a);
        P_global1_1 = rot_a * (P_local1_1 - O1) + O1;
        P_f1_x = P_global1_1(1) + t_k1*sin(theta_k-theta_h-theta_a);
        P_f1_y = P_global1_1(2) + t_k1*cos(theta_k-theta_h-theta_a);
        P_f1 = [P_f1_x; P_f1_y];
        plot([P_global1_1(1), P_f1(1)], [P_global1_1(2), P_f1(2)], 'k-', 'LineWidth', 3);

        % DF spring foot-fixed point
        dir = (P1 - P0) / norm(P1 - P0);
        P_local1_2 = P0 + dir * (d_k1_2+d_a);
        P_global1_2 = rot_a * (P_local1_2 - O1) + O1;
        P_f2_x = P_global1_2(1) + t_k1*sin(theta_k-theta_h-theta_a);
        P_f2_y = P_global1_2(2) + t_k1*cos(theta_k-theta_h-theta_a);
        P_f2 = [P_f2_x; P_f2_y];
        plot([P_global1_2(1), P_f2(1)], [P_global1_2(2), P_f2(2)], 'k-', 'LineWidth', 3);

        % Ankle link on foot
        P_local2 = P0 + dir * d_a;
        P_global2 = rot_a * (P_local2 - O1) + O1;
        plot([P_global2(1), O1(1)], [P_global2(2), O1(2)], 'k-', 'LineWidth', 3);

        % Transmission pivot and screw
        P_t = [O_GRF(1)-(h_f-h_m) * sin(theta_k-theta_h);
               O_GRF(2)-(h_f-h_m) * cos(theta_k-theta_h)];
        plot(P_t(1), P_t(2), 'ko', 'MarkerFaceColor', 'w');
        plot([P_t(1), O1(1)], [P_t(2), O1(2)], 'g-', 'LineWidth', 3);

        P_m_x = P_t(1) + l_g1*cos(theta_k-theta_h-theta_a);
        P_m_y = P_t(2) - l_g1*sin(theta_k-theta_h-theta_a);
        P_m1 = [P_m_x;P_m_y];

        P_m_x = P_t(1) + (x_m+l_g1)*cos(theta_k-theta_h-theta_a);
        P_m_y = P_t(2) - (x_m+l_g1)*sin(theta_k-theta_h-theta_a);
        P_m2 = [P_m_x;P_m_y];

        P_m_x = P_t(1) + (x_m+l_g1+l_g2)*cos(theta_k-theta_h-theta_a);
        P_m_y = P_t(2) - (x_m+l_g1+l_g2)*sin(theta_k-theta_h-theta_a);
        P_m3 = [P_m_x;P_m_y];

        P_m_x = P_t(1) + (x_m+l_g1+l_g2+l_g3)*cos(theta_k-theta_h-theta_a);
        P_m_y = P_t(2) - (x_m+l_g1+l_g2+l_g3)*sin(theta_k-theta_h-theta_a);
        P_m4 = [P_m_x;P_m_y];

        plot([P_t(1), P_m1(1)], [P_t(2), P_m1(2)], 'k-', 'LineWidth', 3);
        plot([P_m1(1), P_m2(1)], [P_m1(2), P_m2(2)], 'y-', 'LineWidth', 3);
        plot([P_m2(1), P_m3(1)], [P_m2(2), P_m3(2)], 'k-', 'LineWidth', 3);
        plot([P_f1(1), P_m3(1)], [P_f1(2), P_m3(2)], 'r--', 'LineWidth', 3);
        plot([P_m3(1), P_m4(1)], [P_m3(2), P_m4(2)], 'k-', 'LineWidth', 3);
        plot([P_m4(1), P_f2(1)], [P_m4(2), P_f2(2)], 'r--', 'LineWidth', 3);

        % Sleeve projections
        P_f3_x = P_m3(1) - t_k1*sin(theta_k-theta_h-theta_a);
        P_f3_y = P_m3(2) - t_k1*cos(theta_k-theta_h-theta_a);
        P_f3 = [P_f3_x; P_f3_y];
        plot([P_m3(1), P_f3(1)], [P_m3(2), P_f3(2)], 'c-', 'LineWidth', 3);

        P_f4_x = P_m4(1) - t_k1*sin(theta_k-theta_h-theta_a);
        P_f4_y = P_m4(2) - t_k1*cos(theta_k-theta_h-theta_a);
        P_f4 = [P_f4_x; P_f4_y];
        plot([P_m4(1), P_f4(1)], [P_m4(2), P_f4(2)], 'c-', 'LineWidth', 3);

        % Spring endpoints markers
        plot(P_m3(1), P_m3(2), 'ko', 'MarkerFaceColor', 'm');
        plot(P_m4(1), P_m4(2), 'ko', 'MarkerFaceColor', 'm');
        plot(P_f1(1), P_f1(2), 'ko', 'MarkerFaceColor', 'm');
        plot(P_f2(1), P_f2(2), 'ko', 'MarkerFaceColor', 'm');

        % Heel spring
        P_top = [O_GRF(1)-(h_f-h_k3) * sin(theta_k-theta_h)+(-d_r-d_d+d_k3) * cos(theta_k-theta_h);
                 O_GRF(2)-(h_f-h_k3) * cos(theta_k-theta_h)-(-d_r-d_d+d_k3) * sin(theta_k-theta_h)];          
        local_s3 = [-(h_a) * sin(theta_k-theta_h)+(-d_r-d_d+d_k3) * cos(theta_k-theta_h);
                    -(h_a) * cos(theta_k-theta_h)-(-d_r-d_d+d_k3) * sin(theta_k-theta_h)];          

        P_bot = rot_a * local_s3 + O1;
        plot([P_top(1), P_bot(1)], [P_top(2), P_bot(2)], 'r--', 'LineWidth', 2);
        plot([P_top(1), O_GRF(1)+(-d_r-d_d+d_k3) * cos(theta_k-theta_h)], ...
             [P_top(2), O_GRF(2)-(-d_r-d_d+d_k3) * sin(theta_k-theta_h)], 'k-', 'LineWidth', 3);

        plot(P_bot(1), P_bot(2), 'ko', 'MarkerFaceColor', 'm');
        plot(P_top(1), P_top(2), 'ko', 'MarkerFaceColor', 'm');

        % Toe link and segment
        O2_local  = [O_GRF(1)+(-h_f+h_t) * sin(theta_k-theta_h)+(-d_r-d_d+l_k) * cos(theta_k-theta_h);
                     O_GRF(2)+(-h_f+h_t) * cos(theta_k-theta_h)-(-d_r-d_d+l_k) * sin(theta_k-theta_h)]; 
        O2_local2 = [O_GRF(1)+(-h_f) * sin(theta_k-theta_h)+(-d_r-d_d+l_k) * cos(theta_k-theta_h);
                     O_GRF(2)+(-h_f) * cos(theta_k-theta_h)-(-d_r-d_d+l_k) * sin(theta_k-theta_h)]; 
        O2   = rot_a * (O2_local  - O1) + O1;
        O2_2 = rot_a * (O2_local2 - O1) + O1;
        
        toe_length = l_t;
        rot_t = [cos(theta_a-theta_k+theta_h+theta_t), -sin(theta_a-theta_k+theta_h+theta_t); ...
                 sin(theta_a-theta_k+theta_h+theta_t),  cos(theta_a-theta_k+theta_h+theta_t)];
        toe_vec = rot_t * [toe_length; 0];
        toe_end = O2_2 + toe_vec;

        plot([O2_2(1), O2(1)], [O2_2(2), O2(2)], 'k-', 'LineWidth', 3);
        plot([O2_2(1), toe_end(1)], [O2_2(2), toe_end(2)], 'b-', 'LineWidth', 3);
        plot(O2(1), O2(2), 'ko', 'MarkerFaceColor', 'w');
        draw_circle(O2, 0.01, 'k', 1);
        draw_circle(O2, 0.006, 'r', 1);

        axis equal;
        axis([-0.5 0.5 -1 0]);
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

% Net ankle torque from GRF about ankle during 0–50% GC (heel friction ignored)
function tau_GRF = ground_reaction_torque(t)
    feet_length = 0.3;    % m
    Body_weight = 60;     % kg
    T_stance = 0.5;       % s

    r_x = [10 15 22.5 27.5 30 32.5 37.5 42 45 51 56 60 65 70 72.5 74.5 76];
    x_old = linspace(0, T_stance, length(r_x));
    t_interp = linspace(0, T_stance, 10);
    r_x_interp = interp1(x_old, r_x, t_interp, 'linear');
    r_x_net = r_x_interp - 25;                 % ankle at 25%
    r_x_actual = r_x_net * feet_length / 100;  % m

    F_y = [0 9 12 11.5 10.5 9.5 9 9.5 10.5 12 11];
    F_y_interp = interp1(linspace(0, T_stance, length(F_y)), F_y, t_interp);
    F_y_actual = F_y_interp * Body_weight;

    tau_half = r_x_actual .* F_y_actual;
    tau_GRF = zeros(size(t));
    idx = t <= T_stance;
    tau_GRF(idx) = interp1(t_interp, tau_half, t(idx), 'linear', 0);
end

% Toe GRF torque about toe/forefoot during 50–60% GC
function tau_toe_GRF = toe_ground_reaction_torque(t)
    feet_length = 0.3; 
    Body_weight = 60; 
    T_start = 0.5; 
    T_stance = 0.1; 

    t_interp = linspace(T_start, T_start + T_stance, 10);
    r_x = [79 83 91];
    t_r = linspace(T_start, T_start + T_stance, length(r_x));
    r_x_interp = interp1(t_r, r_x, t_interp, 'linear');

    r_x_net = r_x_interp - 75;
    r_x_actual = r_x_net * feet_length / 100;

    F_y = [9 5 1];
    t_f = linspace(T_start, T_start + T_stance, length(F_y));
    F_y_interp = interp1(t_f, F_y, t_interp, 'linear');
    F_y_actual = F_y_interp * Body_weight;

    tau_toe_half = r_x_actual .* F_y_actual;

    tau_toe_GRF = zeros(size(t));
    idx = (t >= T_start) & (t <= T_start + T_stance);
    t_local = t(idx) - T_start;
    tau_toe_GRF(idx) = interp1(linspace(0, T_stance, numel(tau_toe_half)), tau_toe_half, t_local, 'linear', 0);
end

% Estimated leg torque about ankle (sign convention consistent with above)
function tau_leg_GRF = leg_ground_reaction_torque(t)
    Body_weight = 60;  % kg
    T_stance = 1;      % s
    t_interp = linspace(0, T_stance, 20);

    r_x = [-5  -2.5   0    1    2    4   6   8   10   12  13 14  15 0 0 0 0 0 0 0 0]/100;
    F_y = [0     9    12  11.5 10.5 9.5 9 9.5 10.5 12 11 6 1 0 0 0 0 0 0 0 0];
    r_x_interp = interp1(linspace(0, T_stance, length(r_x)), r_x, t_interp);
    F_y_interp = interp1(linspace(0, T_stance, length(F_y)), F_y, t_interp);
    F_y_actual = F_y_interp * Body_weight;

    tau_leg_half = r_x_interp .* F_y_actual;
    tau_leg_GRF = zeros(size(t));
    idx = t <= T_stance;
    tau_leg_GRF(idx) = interp1(t_interp, tau_leg_half, t(idx), 'linear', 0);
end

% Reaction torque from toe on ankle during push-off (50–60% GC)
function toe_ankle = toe_ankle_ground_reaction_torque(t)
    T_start = 0.5; 
    T_stance = 0.1;             
    t_interp = linspace(0, T_stance, length([70 40 20]));
    M_toe_ankle = [70 40 20];
    toe_ankle = zeros(size(t));
    idx = (t >= T_start) & (t <= T_start + T_stance);
    t_local = t(idx) - T_start;
    toe_ankle(idx) = interp1(t_interp, M_toe_ankle, t_local, 'linear', 0);
end

function tau = pyStyleCross2D(a, b)
    % z-component of 3-D cross([a 0],[b 0])
    cross_3d = cross([a(1), a(2), 0], [b(1), b(2), 0]);
    tau = cross_3d(3);
end


