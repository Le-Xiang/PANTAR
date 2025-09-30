function prosthetic_foot_sim_v7()
    % Parameter definitions
    I_a = 0.01;      % Foot plate moment of inertia (kg·m^2)
    I_t = 0.0001;     % Toe joint moment of inertia (kg·m^2)
    m_t = 0.08;      % Toe mass (kg)
    d = 0.16;        % Distance from toe joint to ankle joint (m)
    eta = 0.9;       % Efficiency factor η 

    % There should be other moments of inertia to add, because there are motors and other parts on the foot plate, not yet included
    I_toe_at_ankle = I_t + m_t * d^2;
    I_a_total = I_a + I_toe_at_ankle;

    k1 = 100000; L0 = 0.04;
    n = 3; 
    p_m_per_rev = 0.01; % 5 mm/rev = 0.005 m/rev. This needs to consider torque: the larger it is, the less torque it can withstand, but the slower the response. If too large, it may cause overshoot.
    p = p_m_per_rev / (2 * pi);  % m/rad
    k3 = 50; L3 = 0.12;
    n_t = 5; theta_t0 = 0;
    I_actual = 0; % Should be the actual measured motor current, external input variable

    % Damping parameters
    b_a = 0.05;       % Ankle joint damping N·m·s/rad
    b_t = 0.01;       % Toe joint damping N·m·s/rad
    b_m = 0.005;      % Motor shaft (toe joint) damping N·m·s/rad

    % Initial state:
    % Add 4 error integrals and previous error variables: ankle joint integral error, previous ankle error, toe joint integral error, previous toe error
    y0 = [0; 0; 0.03; 0; 0; 0; 0; 0]; 

    tspan = [0 1];

    [t, y] = ode45(@(t,y) dynamics_v7(t, y, ...
        I_a_total, I_t, k1, L0, n, p, k3, L3, ...
        n_t, theta_t0, b_a, b_t, b_m, eta, I_actual), tspan, y0);

    % Plot: ankle angle tracking performance
    figure();
    plot(t, y(:,1)*180/pi, 'LineWidth', 2); hold on;
    plot(t, arrayfun(@ankle_trajectory_real, t)*180/pi, '--r', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    title('Ankle joint angle tracking (solid: actual, dashed: desired)');
    legend('Actual angle', 'Desired trajectory');
    grid on;

    % Plot: toe joint angle tracking performance
    figure();
    plot(t, y(:,5)*180/pi, 'LineWidth', 2); hold on;
    plot(t, arrayfun(@toe_trajectory_real, t)*180/pi, '--r', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    title('Toe joint angle tracking (solid: actual, dashed: desired)');
    legend('Actual angle', 'Desired trajectory');
    grid on;
    
    figure();
    visualize_motion_v7(t, y);
end

function dydt = dynamics_v7(t, y, I_a_total, I_t, ...
    k1, L0, n, p, k3, L3, n_t, theta_t0, b_a, b_t, b_m, eta, I_actual)

    theta_a = y(1); omega_a = y(2);
    x_m = y(3); omega_m = y(4);  % Motor angular velocity
    theta_t = y(5); omega_t = y(6);
    integral_e_a = y(7);
    integral_e_t = y(8);

    % Desired trajectory
    theta_a_ref = ankle_trajectory_real(t);
    theta_t_ref = toe_trajectory_real(t);

    % --- Ankle joint PID, calculate desired motor speed ---
    e_a = theta_a_ref - theta_a;
    omega_a_ref = 0; % omega_a_ref = 0 usually holds (if you just want the ankle to reach a fixed point)
    de_a = omega_a_ref - omega_a;  % Use speed error to estimate error derivative

    Kp_a = 150; Ki_a = 20; Kd_a = 20;
    omega_m_ref = Kp_a*e_a + Ki_a*integral_e_a + Kd_a * de_a;

    % --- Motor speed control, calculate motor angular acceleration ---
    K_motor = 5; % Motor speed loop gain
    alpha_m = K_motor * (omega_m_ref - omega_m);

    % Another way: PID control

    % --- Toe joint PID ---
    e_t = theta_t_ref - theta_t;

    Kp_t = 100; Ki_t = 1;
    tau_pid_t = Kp_t*e_t + Ki_t*integral_e_t;

    % Convert torque to target current
    k_t = 0.5;
    I_ref = tau_pid_t / k_t;
    
    % Inner controller: current PI control (using current sensor feedback)
    error_I = I_ref - I_actual;
    % voltage_out = PI_current_control(error_I); % Control function to be extended
    
    % Output PWM or voltage to control motor
    % set_motor_voltage(voltage_out); % PWM output to be extended

    % --- Spring and damping calculations remain unchanged --- counterclockwise is positive by default
    O1 = [0; -0.05]; 

    rot_a = [cos(theta_a), -sin(theta_a); sin(theta_a), cos(theta_a)];
    % local_s1 = [0.08; -0.03];
    % P_f = rot_a * local_s1 + O1;
    P0 = [-0.03; -0.12];
    P1 = [0.16; -0.12];
    dir = (P1 - P0) / norm(P1 - P0);
    P_local = P0 + dir * 0.11;
    P_global = rot_a * (P_local - O1) + O1;

    P_f_x = P_global(1) - 0.03*sin(theta_a);
    P_f_y = P_global(2) + 0.03*cos(theta_a);
    P_f = [P_f_x,P_f_y];

    % P_m = [x_m; -0.09];
    P_c = [0.01,-0.09];
    v = P_f - P_c;
    theta_b = atan2(v(2), v(1));
    P_m_x = 0.01 + x_m*cos(theta_b);
    P_m_y = -0.09 + x_m*sin(theta_b);
    P_m = [P_m_x,P_m_y];

    spring_vec = P_f - P_m;
    spring_len = norm(spring_vec);
    fprintf('t=%.3f, spring_len=%.4f\n', t, spring_len);
    spring_dir = spring_vec / spring_len;
    F_spring1 = k1 * (spring_len - L0) * spring_dir;
    r1 = P_f - O1;
    torque1 = cross2D(r1, F_spring1);

    P_top = [-0.03; -0.03];
    local_s3 = [-0.035; -0.07];
    P_bottom = rot_a * local_s3 + O1;
    P_bottom_x = P_bottom(1) - 0.03*sin(theta_a);
    P_bottom_y = P_bottom(2) + 0.03*cos(theta_a);
    P_bottom = [P_bottom_x,P_bottom_y];
    spring_vec3 = P_top - P_bottom;
    spring_len3 = norm(spring_vec3);
    spring_dir3 = spring_vec3 / spring_len3;
    F_spring3 = k3 * (spring_len3 - L3) * spring_dir3;
    r3 = P_bottom - O1;
    torque3 = cross2D(r3, F_spring3);

    % Calculate total ankle joint torque, including PID output, spring, damping, and support torque
    tau_GRF = ground_reaction_torque(t);
    % total_torque_a = torque1 + torque3 - b_a * omega_a + tau_GRF;
    total_torque_a = torque1 + torque3 - b_a * omega_a;
    % Note: The motor torque of the ankle joint is changed from PID to speed loop control, not directly added here

    % Toe joint torque remains unchanged
    torque_spring_t = -2 * (theta_t - theta_t0); % k2 = 2
    damping_joint_t = -b_t * omega_t;
    damping_motor_t = - (b_m / n_t^2) * omega_t;
    tau_toe_GRF = toe_ground_reaction_torque(t);
    % total_torque_t = tau_pid_t / n_t + torque_spring_t + damping_joint_t + damping_motor_t + tau_toe_GRF;
    total_torque_t = tau_pid_t / n_t + torque_spring_t + damping_joint_t + damping_motor_t + tau_toe_GRF;

    % State derivative update
    dydt = zeros(8,1);
    dydt(1) = omega_a;
    dydt(2) = total_torque_a / I_a_total;

    dx_m = omega_m * n * p;    % Motor lead screw position change, p is lead screw pitch
    
    % Limit logic (limit derivative! not state) determined by lead screw range
    x_min = 0; x_max = 0.07;
    if y(3) <= x_min && dx_m < 0
        dx_m = 0;    % Directly set dx_m = 0, is this correct? Should I only control my independent variable?
        alpha_m = 0; % Should omega_m also be limited at the same time?
    elseif y(3) >= x_max && dx_m > 0
        dx_m = 0;
        alpha_m = 0;
    end
    
    % Limit logic (limit derivative! not state) determined by the position between x_m and P_f. The former must be inside the latter, otherwise it will be pushed out / the right side P_f of the spring system cannot be compressed to 0
    if y(3) >= norm(v) && dx_m > 0
        dx_m = 0;    
        alpha_m = 0;
    elseif norm(v) <= 0
        dx_m = 0;    
        alpha_m = 0;
    end

    dydt(3) = dx_m;
    dydt(4) = alpha_m;        % Motor angular acceleration, generated by speed closed-loop control
    dydt(5) = omega_t;
    dydt(6) = total_torque_t / I_t;
    dydt(7) = e_a;
    dydt(8) = e_t;
end

function theta_ref = ankle_trajectory(t) % -25 +15
    theta_start = 0 * pi/180;
    theta_end = 15 * pi/180;
    T = 1;
    if t <= T
        theta_ref = theta_start + (theta_end - theta_start) * t / T;
    else
        theta_ref = theta_end;
    end
end

function theta_ref = ankle_trajectory_real(t)
    % Simulate normal human ankle joint angle trajectory (black solid line)
    % Input: t ∈ [0, 1] (time, in seconds)
    % Output: theta_ref (unit: rad)

    % Map t ∈ [0,1] to %GC ∈ [0,100]
    gc = t * 100;

    % Original data points (estimated from graph, unit: deg)
    % x_gc =        [0    5  10  15  20 25  30  35 40  45  50 55 60   65   70 75 80 85  90  95 100];
    % y_angle_deg = [-5 -7.5 -5 -2.5 0 1.25 2.5 5  6.5 8.5 10 5  -5 -12.5 -10 -5 -1  0  -2 -3.5 -5 ];
    
    x_gc =        [0    5  10  15  20 25  30  35 40  45  50 55 60   65   70 75 80 85  90  95 100];
    y_angle_deg = [-5 -7.5 -5 -2.5 0 1.25 2.5 5  6.5 8.5 10 5  -5 -12.5 -10 -5 -1  0  -2 -3.5 -5];
                 
    % Spline interpolation and convert to radians
    y_angle_rad = deg2rad(y_angle_deg);
    theta_ref = interp1(x_gc, y_angle_rad, gc, 'pchip');

    % Out-of-bounds protection
    theta_ref(gc < 0 | gc > 100) = 0;
end



function theta_ref = toe_trajectory(t) % -10 +60
    theta_start = 0 * pi/180;
    theta_end = -10 * pi/180;
    T = 1;
    if t <= T
        theta_ref = theta_start + (theta_end - theta_start) * t / T;
    else
        theta_ref = theta_end;
    end
end

function theta_ref = toe_trajectory_real(t)
    % Fit to the blue curve in the first figure
    % Input: t ∈ [0,1] (unit: s)
    % Output: theta_ref (unit: rad)

    % Map t → %GC
    gc = t * 100;

    % Blue line angle data estimated from the graph (unit: deg)
    x_gc =        [0  5 10 15 20 25 30 35 40 45 50 55   60 65 70 75    80   85 90  95 100];
    y_angle_deg = [12 8  4 2 0 1.75 2 3.5  6   9 15 27.5 37 20 10 10.25 10.5 11 11.25 11.5 12];

    % Interpolate & convert to radians
    y_angle_rad = deg2rad(y_angle_deg);
    theta_ref = interp1(x_gc, y_angle_rad, gc, 'pchip');

    % Out-of-bounds protection
    theta_ref(gc < 0 | gc > 100) = 0;
end


function z = cross2D(a, b)
    z = a(1)*b(2) - a(2)*b(1);
end

function visualize_motion_v7(t, y)
    for i = 1:10:length(t)
        theta_a = y(i,1);
        x_m = y(i,3);
        theta_t = y(i,5);

        clf; hold on;

        plot([0, 0], [0, -0.06], 'k-', 'LineWidth', 3);
        O1 = [0; -0.05];
        plot(O1(1), O1(2), 'ko', 'MarkerFaceColor', 'w');
        draw_circle(O1, 0.01, 'k', 1);
        draw_circle(O1, 0.015, 'k', 1);

        plot([0, 0], [-0.06, -0.09], 'k-', 'LineWidth', 3);

        plot([-0.03, 0.03], [0, 0], 'b-', 'LineWidth', 3);
        plot([-0.03, -0.03], [-0.03, 0], 'k-', 'LineWidth', 3);
        plot(-0.03, -0.03, 'ko', 'MarkerFaceColor', 'm');

        foot_pts = [-0.035, -0.12; 0.16, -0.12]';
        rot_a = [cos(theta_a), -sin(theta_a); sin(theta_a), cos(theta_a)];
        foot_rot = rot_a * (foot_pts - O1) + O1;
        plot(foot_rot(1,:), foot_rot(2,:), 'b-', 'LineWidth', 3);

    % P_m = [x_m; -0.09];
    % local_s1 = [0.08; -0.03];
    % P_f = rot_a * local_s1 + O1;

        P0 = [-0.03; -0.12];
        P1 = [0.16; -0.12];
        dir = (P1 - P0) / norm(P1 - P0);
        P_local = P0 + dir * 0.11;
        P_global = rot_a * (P_local - O1) + O1;

        P_f_x = P_global(1) - 0.03*sin(theta_a);
        P_f_y = P_global(2) + 0.03*cos(theta_a);
        P_f = [P_f_x,P_f_y];

        P_c = [0.01,-0.09];
        v = P_f - P_c;
        theta_b = atan2(v(2), v(1));
        P_m_x = 0.01 + x_m*cos(theta_b);
        P_m_y = -0.09 + x_m*sin(theta_b);
        P_m = [P_m_x,P_m_y];

        plot([P_m(1), P_f(1)], [P_m(2), P_f(2)], 'r--', 'LineWidth', 2);

        plot([0.01, P_m(1)], [-0.09, P_m(2)], 'c-', 'LineWidth', 3);
        % plot([P_m(1), P_f(1)], [P_m(2), P_f(2)], 'c-', 'LineWidth', 1);

        plot([0, 0.01], [-0.09, -0.09], 'g-', 'LineWidth', 3);
        plot(0.01, -0.09, 'ko', 'MarkerFaceColor', 'y');

        plot([P_global(1), P_f(1)], [P_global(2), P_f(2)], 'k-', 'LineWidth', 3);

        P_top = [-0.03; -0.03];
        local_s3 = [-0.035; -0.07];
        P_bot = rot_a * local_s3 + O1;
        P_bottom_x = P_bot(1) - 0.03*sin(theta_a);
        P_bottom_y = P_bot(2) + 0.03*cos(theta_a);
        P_bottom = [P_bottom_x,P_bottom_y];
        plot([P_top(1), P_bottom(1)], [P_top(2), P_bottom(2)], 'r--', 'LineWidth', 2);
        
        plot([P_bot(1), P_bottom(1)], [P_bot(2), P_bottom(2)], 'k-', 'LineWidth', 3);
        plot(P_bottom(1), P_bottom(2), 'ko', 'MarkerFaceColor', 'm');

        O2_local = [0.16; -0.12];
        O2 = rot_a * (O2_local - O1) + O1;
        draw_circle(O2, 0.01, 'k', 1);

        toe_length = 0.05;
        rot_t = [cos(theta_t), -sin(theta_t); sin(theta_t), cos(theta_t)];
        toe_vec = rot_t * [toe_length; 0];
        toe_end = O2 + toe_vec;

        plot([O2(1), toe_end(1)], [O2(2), toe_end(2)], 'b-', 'LineWidth', 3);
        plot(O2(1), O2(2), 'ko', 'MarkerFaceColor', 'w');
        draw_circle(O2, 0.006, 'r', 1);


        local_s4 = [-0.015; -0.07];
        P_p = rot_a * local_s4 + O1;
        P_p_x = P_p(1) - 0.07*sin(theta_a);
        P_p_y = P_p(2) + 0.07*cos(theta_a);
        P_p_bottom = [P_p_x,P_p_y];
        plot([P_p(1), P_p_bottom(1)], [P_p(2), P_p_bottom(2)], 'k-', 'LineWidth', 3);

        local_s5 = [0.015; -0.07];
        local_s6 = [0.03; -0.07];
        P_p = rot_a * local_s5 + O1;
        P_p_x = P_p(1) - 0.07*sin(theta_a);
        P_p_y = P_p(2) + 0.07*cos(theta_a);
        P_pp = rot_a * local_s6 + O1;
        P_l_x = P_pp(1) - 0.04*sin(theta_a);
        P_l_y = P_pp(2) + 0.04*cos(theta_a);
        P_p_bottom_1 = [P_p_x,P_p_y];
        P_p_bottom_2 = [P_l_x,P_l_y];
        plot([P_p_bottom_1(1), P_p_bottom_2(1)], [P_p_bottom_1(2), P_p_bottom_2(2)], 'k-', 'LineWidth', 3);
        local_s7 = [0.06; -0.07];
        P_ppp = rot_a * local_s7 + O1;
        P_t_x = P_ppp(1) - 0.04*sin(theta_a);
        P_t_y = P_ppp(2) + 0.04*cos(theta_a);
        P_p_bottom_3 = [P_t_x,P_t_y];
        plot([P_p_bottom_2(1), P_p_bottom_3(1)], [P_p_bottom_2(2), P_p_bottom_3(2)], 'k-', 'LineWidth', 3);
        local_s7 = [0.08; -0.07];
        P_pppp = rot_a * local_s7 + O1;
        plot([P_pppp(1), P_p_bottom_3(1)], [P_pppp(2), P_p_bottom_3(2)], 'k-', 'LineWidth', 3);
    


        axis equal;
        axis([-0.1 0.3 -0.15 0.05]);
        title(sprintf('时间 %.2f s', t(i)));
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

% The heel friction torque can probably be ignored
function tau_GRF = ground_reaction_torque(t)
    % t: input time (unit: s), can be scalar or vector

    % --- Simulation parameter settings ---
    feet_length = 0.25;           % Foot length (m)
    Body_weight = 60;             % Body weight (kg)
    T_stance = 0.5;               % Stance phase duration (s)

    % --- Original r_x data (assumed unit: % foot length) ---
    r_x = [10 15 22.5 27.5 30 32.5 37.5 42 45 51 56 60 65 70 72.5 74.5 76];
    x_old = linspace(0, T_stance, length(r_x));   % Corresponds to stance phase 0~0.5s

    % --- Interpolate to 10 time points (or more) ---
    t_interp = linspace(0, T_stance, 10);
    r_x_interp = interp1(x_old, r_x, t_interp, 'linear');
    r_x_net = r_x_interp - 25;                    % Centered at ankle (25%)
    r_x_actual = r_x_net * feet_length / 100;     % Convert to meters

    % --- Simulate vertical GRF, unit: BW (body weight) ratio ---
    F_y = [0 9 12 11.5 10.5 9.5 9 9.5 10.5 12 11]; % 11个点
    F_y_interp = interp1(linspace(0, T_stance, length(F_y)), F_y, t_interp);
    F_y_actual = F_y_interp * Body_weight;

    % --- Calculate torque vector for the first 0.5 seconds ---
    tau_half = r_x_actual .* F_y_actual;  % Cross product: r × F (scalar)

    % --- Output tau at corresponding time points ---
    tau_GRF = zeros(size(t));             % Initialize to 0

    % Interpolate for t in [0, 0.5]
    idx = t <= T_stance;
    tau_GRF(idx) = interp1(t_interp, tau_half, t(idx), 'linear', 0);  % Interpolate, 0 if out of range
end

function tau_toe_GRF = toe_ground_reaction_torque(t)
    % t: input time (unit: s), can be scalar or vector

    % --- Simulation parameter settings ---
    feet_length = 0.25;           % Foot length (m)
    Body_weight = 60;             % Body weight (kg)
    T_start = 0.5;                % Toe support phase start time
    T_stance = 0.1;               % Toe support phase duration (s)

    % --- Time nodes and interpolation time ---
    t_interp = linspace(T_start, T_start + T_stance, 10);  % Toe support period: 0.5~0.6s

    % --- Original r_x data (% foot length) ---
    r_x = [79 83 91];  % Force application point moves in the toe direction
    t_r = linspace(T_start, T_start + T_stance, length(r_x));
    r_x_interp = interp1(t_r, r_x, t_interp, 'linear');

    % --- Relative to ankle joint position (set ankle at 25%) ---
    r_x_net = r_x_interp - 75;
    r_x_actual = r_x_net * feet_length / 100;  % Convert to meters

    % --- Vertical GRF (unit: body weight ratio) ---
    F_y = [9 5 1];
    t_f = linspace(T_start, T_start + T_stance, length(F_y));
    F_y_interp = interp1(t_f, F_y, t_interp, 'linear');
    F_y_actual = F_y_interp * Body_weight;

    % --- Calculate torque: tau = r × F (scalar multiplication in 2D) ---
    tau_toe_half = r_x_actual .* F_y_actual;

    % --- Output final result ---
    tau_toe_GRF = zeros(size(t));  % Initialize

    % Find indices in t that are within the toe support time period
    idx = (t >= T_start) & (t <= T_start + T_stance);

    % Interpolate output
    tau_toe_GRF(idx) = interp1(t_interp, tau_toe_half, t(idx), 'linear', 0);
end 
