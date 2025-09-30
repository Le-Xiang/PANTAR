function prosthetic_foot_sim_v7()
    spring_len_array = [];
    assignin('base', 'spring_len_array', spring_len_array);
    spring_len3_array = [];
    assignin('base', 'spring_len3_array', spring_len3_array);
    x_m_array = [];
    assignin('base', 'x_m_array', x_m_array);
    omega_m_ref_array = [];
    assignin('base', 'omega_m_ref_array', omega_m_ref_array);
    omega_m_array = [];
    assignin('base', 'omega_m_array', omega_m_array);
    omega_t_array = [];
    assignin('base', 'omega_t_array', omega_t_array);
    tau_pid_t_array = [];
    assignin('base', 'tau_pid_t_array', tau_pid_t_array);

    % Parameter definitions
    I_a = 0.01;      % Foot plate moment of inertia (kg·m^2)
    I_t = 0.0001;    % Toe joint moment of inertia (kg·m^2)
    m_t = 0.08;      % Toe mass (kg)
    d = 0.20;        % Distance from toe joint to ankle joint (m)
    eta = 0.9;       % Efficiency factor η 

    % There should be other moments of inertia to add (motors and other parts on the foot plate),
    % not yet included. (Should be considered during swing and stance except the sub-phase from full
    % foot contact to forefoot contact to toe-off.)
    I_toe_at_ankle = I_t + m_t * d^2;
    I_a_total = I_a + I_toe_at_ankle;

    % In the excluded process, the ankle joint doesn’t need to consider toe rotation (decoupled)
    % I_a_total = I_a;

    % System 1 - Ankle joint
    k1 = 212000;      % Spring k1 stiffness
    L0 = 0.04;        % Spring k1 free length 0.04 m
    n = 4;            % Gear ratio of k1 system
    p_m_per_rev = 0.01; % Leadscrew pitch (m/rev). Larger → less torque capacity, smaller → slower.
                        % Too large may cause overshoot. Final choice used here: 0.01 m/rev (10 mm).

    p = p_m_per_rev / (2 * pi);  % m/rad
    theta_a0 = deg2rad(0);       % Initial ankle angle

    % System 3 - Ankle joint
    k3 = 670;     % Spring k3 stiffness
    L3 = 0.0602;  % Spring k3 free length

    % System 2 - Toe joint
    n_t = 5;               % Gear ratio of k2 system
    theta_t0 = deg2rad(12);% Initial toe angle
    I_actual = 0;          % Actual measured motor current (external input)
    k2 = 1.075;            % Spring k2 stiffness

    % Damping parameters
    b_a = 0.05;   % Ankle damping N·m·s/rad
    b_t = 0.01;   % Toe joint damping N·m·s/rad
    b_m = 0.005;  % Motor shaft (toe) damping N·m·s/rad

    % Initial state:
    % Add 4 error integrals and previous error variables: ankle integral error,
    % previous ankle error, toe integral error, previous toe error
    y0 = [theta_a0; 0; 0.03; 0; theta_t0; 0; 0; 0]; 

    tspan = [0 1];
    % tspan = linspace(0, 1, 1000);
    % options = odeset('RelTol', 1e-8, 'AbsTol', 1e-8);  % improve solver accuracy

    [t, y] = ode45(@(t,y) dynamics_v7(t, y, ...
        I_a_total, I_t, k1, L0, n, p, k3, L3, ...
        n_t, b_a, b_t, b_m, eta, I_actual, k2), tspan, y0);

    % [t, y] = ode45(@(t,y) dynamics_v7(t, y, ...
    %     I_a_total, I_t, k1, L0, n, p, k3, L3, ...
    %     n_t, b_a, b_t, b_m, eta, I_actual, k2), tspan, y0, options);

    % Plot: ankle angle tracking performance
    figure();
    plot(t, y(:,1)*180/pi, 'LineWidth', 2); hold on;
    plot(t, arrayfun(@ankle_trajectory_real, t)*180/pi, '--r', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    title('Ankle angle tracking (solid: actual, dashed: desired)');
    legend('Actual angle', 'Desired trajectory');
    grid on;

    % Plot: toe joint angle tracking performance
    figure();
    plot(t, y(:,5)*180/pi, 'LineWidth', 2); hold on;
    plot(t, arrayfun(@toe_trajectory_real, t)*180/pi, '--r', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    title('Toe angle tracking (solid: actual, dashed: desired)');
    legend('Actual angle', 'Desired trajectory');
    grid on;
   
    figure();
    visualize_motion_v7(t, y);

    spring_len_array = evalin('base', 'spring_len_array');
    fprintf('min spring len = %.4f m\n', min(spring_len_array));
    fprintf('max spring len = %.4f m\n', max(spring_len_array));
    spring_len3_array = evalin('base', 'spring_len3_array');
    fprintf('min spring len3 = %.4f m\n', min(spring_len3_array));
    fprintf('max spring len3 = %.4f m\n', max(spring_len3_array));
    x_m_array = evalin('base', 'x_m_array');
    fprintf('min x m = %.4f m\n', min(x_m_array));
    fprintf('max x m = %.4f m\n', max(x_m_array));
    omega_m_ref_array = evalin('base', 'omega_m_ref_array');
    fprintf('min omega m ref = %.4f rad/s\n', min(abs(omega_m_ref_array)));
    fprintf('max omega m ref = %.4f rad/s\n', max(abs(omega_m_ref_array)));
    Q0 = prctile(abs(omega_m_ref_array), 97.5);
    fprintf('Q0 (97.5%%) = %.4f rad/s\n', Q0);
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
end

function dydt = dynamics_v7(t, y, I_a_total, I_t, ...
    k1, L0, n, p, k3, L3, n_t, b_a, b_t, b_m, eta, I_actual, k2)

    theta_a = y(1); % Ankle joint angle
    omega_a = y(2); % Ankle joint angular velocity
    x_m = y(3);     % Ball screw nut position
    omega_m = y(4); % Motor angular velocity
    theta_t = y(5); % Toe joint angle
    omega_t = y(6); % Toe joint angular velocity
    integral_e_a = y(7); 
    integral_e_t = y(8);

    % Desired trajectory
    theta_a_ref = ankle_trajectory_real(t);
    theta_t_ref = toe_trajectory_real(t);
    omega_a_ref = ankle_angular_velocity_real(t);

    % --- Ankle joint PID: compute desired motor speed ---
    e_a = theta_a_ref - theta_a;
    % omega_a_ref = 0 usually holds if targeting a fixed point
    de_a = omega_a_ref - omega_a;  % Use speed error to approximate error derivative

    Kp_a = 400; Ki_a = 10; Kd_a = 17.5;
    % max_val = 10;  % anti-windup clamp example
    % integral_e_a = max(min(integral_e_a, max_val), -max_val);

    omega_m_ref = (Kp_a*e_a + Ki_a*integral_e_a + Kd_a * de_a);
    % omega_m_ref = (Kp_a*e_a + Ki_a*integral_e_a);
    fprintf('t=%.2f, e_a=%.4f, int_e_a=%.4f, Ki*int=%.4f\n', ...
    t, e_a, integral_e_a, Ki_a*integral_e_a);

    fprintf('t=%.3f, omega_m_ref=%.4f', ...
    t, omega_m_ref);
    fprintf('t=%.3f, x_m=%.4f,e_a=%.4f', ...
    t, x_m, e_a);

    % --- Motor speed control: compute motor angular acceleration ---
    K_motor = 5; % Motor speed loop gain
    alpha_m = K_motor * (omega_m_ref - omega_m);
    % Another way: full PID speed loop

    % --- Toe joint PID ---
    e_t = theta_t_ref - theta_t;
    Kp_t = 800; Ki_t = 20;
    tau_pid_t = Kp_t*e_t + Ki_t*integral_e_t;

    % Convert torque to target current
    k_t = 0.5;
    I_ref = tau_pid_t / k_t;
    % Inner controller: current PI control with sensor feedback
    % error_I = I_ref - I_actual;
    % voltage_out = PI_current_control(error_I);
    % set_motor_voltage(voltage_out);

    % --- Spring and damping calculations (counterclockwise positive) ---
    O1 = [0; -0.05]; % Ankle joint coordinates

    % Ankle rotation
    rot_a = [cos(theta_a), -sin(theta_a); sin(theta_a), cos(theta_a)];
    P0 = [-0.05; -0.17]; % Foot plate left endpoint (initial, ankle=0)
    P1 = [0.24; -0.17];  % Foot plate right endpoint (initial, ankle=0)
    dir = (P1 - P0) / norm(P1 - P0);
    P_local = P0 + dir * 0.115;
    P_global = rot_a * (P_local - O1) + O1; % Spring k1 end fixed on foot plate

    % Spring k1 end position
    P_f_x = P_global(1) - 0.01*sin(theta_a);
    P_f_y = P_global(2) + 0.01*cos(theta_a);
    P_f = [P_f_x,P_f_y]; 

    P_c = [0.00,-0.11]; % Left endpoint of the coupling

    v = P_f - P_c;
    theta_b = atan2(v(2), v(1)); % Angle between screw and horizontal
    P_m_x = P_c(1) + (x_m+0.01)*cos(theta_b);
    P_m_y = P_c(2) + (x_m+0.01)*sin(theta_b);
    P_m = [P_m_x,P_m_y]; % Spring k1 start / ball screw nut position

    P_c_x = P_c(1) + 0.01*cos(theta_b);
    P_c_y = P_c(2) + 0.01*sin(theta_b);
    P_c_r = [P_c_x,P_c_y];

    % k1 torque on ankle
    spring_vec = P_f - P_m;
    spring_len = norm(spring_vec);
    spring_dir = spring_vec / spring_len;
    F_spring1 = k1 * (L0 - spring_len) * spring_dir;
    F1 = norm(F_spring1);
    r1 = P_f - O1; 
    % Torque of spring k1 on ankle joint. Sign may look inverted due to coordinate conventions.
    torque1 = -cross2D(r1, F_spring1);
    % torque1 = pyStyleCross2D(r1, F_spring1);

    % Spring system k3
    P_top = [-0.03; -0.03];
    local_s3 = [-0.035; -0.07];
    P_bottom = rot_a * local_s3 + O1;
    P_bottom_x = P_bottom(1) - 0.03*sin(theta_a);
    P_bottom_y = P_bottom(2) + 0.03*cos(theta_a);
    P_bottom = [P_bottom_x,P_bottom_y];
    spring_vec3 = P_top - P_bottom;
    spring_len3 = norm(spring_vec3);
    spring_dir3 = spring_vec3 / spring_len3;
    % Note: if you see anomalies in spring_len3, double-check geometry
    F_spring3 = k3 * (spring_len3 - L3) * spring_dir3;
    F3 = norm(F_spring3);
    r3 = P_bottom - O1;
    torque3 = cross2D(r3, F_spring3);

    fprintf(['t=%.3f, torque1=%.4f, torque3=%.4f, spring_len=%.4f, spring_len3=%.4f, ', ...
             'F1=%.4f, F3=%.4f, tau_pid_t = %.4f, omega_t = %.4f\n  '], ...
            t, torque1, torque3, spring_len, spring_len3, F1, F3, tau_pid_t, omega_t);

    % Total ankle torque: springs, damping, GRF/support torques
    toe_ankle = toe_ankle_ground_reaction_torque(t);
    tau_GRF = ground_reaction_torque(t);
    tau_leg = leg_ground_reaction_torque(t);
    total_torque_a = torque1 + torque3 - b_a * omega_a + tau_GRF - tau_leg + toe_ankle;
    % total_torque_a = torque1 + torque3 - b_a * omega_a;
    % Note: Ankle motor torque is controlled by a speed loop (not added directly here)

    % Toe joint torque
    torque_spring_t = -k2 * (theta_t - 0);
    damping_joint_t = -b_t * omega_t;
    damping_motor_t = - (b_m / n_t^2) * omega_t;
    tau_toe_GRF = toe_ground_reaction_torque(t);
    total_torque_t = tau_pid_t / n_t + torque_spring_t + damping_joint_t + damping_motor_t + tau_toe_GRF;

    % State derivatives
    dydt = zeros(8,1);
    dydt(1) = omega_a;
    dydt(2) = total_torque_a / I_a_total;

    dx_m = omega_m * n * p;    % Leadscrew position derivative; p is pitch (m/rad)
    fprintf('t=%.3f, dx_m=%.4f', t, dx_m);
    
    % Limit logic (limit derivatives, not states): leadscrew range
    x_min = 0; x_max = 0.07;
    if y(3) <= x_min && dx_m < 0
        dx_m = 0;
        alpha_m = 0; % Also clamp motor accel
    elseif y(3) >= x_max && dx_m > 0
        dx_m = 0;
        alpha_m = 0;
    end
    
    % Limit logic: geometric constraint between x_m and P_f.
    % The nut must remain inside the segment; prevent over-compression to zero length.
    if y(3) >= norm(v) && dx_m > 0
        dx_m = 0;    
        alpha_m = 0;
    elseif norm(v) <= 0
        dx_m = 0;    
        alpha_m = 0;
    end

    dydt(3) = dx_m;
    dydt(4) = alpha_m;        % Motor angular acceleration (from speed loop)
    dydt(5) = omega_t;
    dydt(6) = total_torque_t / I_t;
    dydt(7) = e_a;
    dydt(8) = e_t;

    assignin('base', 'spring_len_array', [evalin('base','spring_len_array'); spring_len]);
    assignin('base', 'spring_len3_array', [evalin('base','spring_len3_array'); spring_len3]);
    assignin('base', 'x_m_array', [evalin('base','x_m_array'); x_m]);
    assignin('base', 'omega_m_ref_array', [evalin('base','omega_m_ref_array'); omega_m_ref]);
    assignin('base', 'omega_m_array', [evalin('base','omega_m_array'); omega_m]);
    assignin('base', 'omega_t_array', [evalin('base','omega_t_array'); omega_t]);
    assignin('base', 'tau_pid_t_array', [evalin('base','tau_pid_t_array'); tau_pid_t]);
end

function theta_ref = ankle_trajectory(t) % -25 to +15 deg
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
    % Simulated normal human ankle angle trajectory (black solid line)
    % Input: t ∈ [0, 1] s
    % Output: theta_ref (rad)

    % Map t ∈ [0,1] to %GC ∈ [0,100]
    gc = t * 100;

    % Estimated from plot (deg)
    x_gc =        [0    5  10  15  20 25  30  35 40  45  50 55 60   65   70 75 80 85  90  95 100];
    y_angle_deg = [-5 -7.5 -5 -2.5 0 1.25 2.5 5  6.5 8.5 10 5  -5 -12.5 -10 -5 -1  0  -2 -3.5 -5];

    % Spline interpolation and convert to radians
    y_angle_rad = deg2rad(y_angle_deg);
    theta_ref = interp1(x_gc, y_angle_rad, gc, 'pchip');

    % Out-of-range protection
    theta_ref(gc < 0 | gc > 100) = 0;
end

function omega_ref = ankle_angular_velocity_real(t)
    dt = 0.001;  % Differencing step; adjust per control frequency
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

function theta_ref = toe_trajectory(t) % -10 to +60 deg
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
    % Input: t ∈ [0,1] s
    % Output: theta_ref (rad)

    % Map t → %GC
    gc = t * 100;

    % Blue-line angle data (deg), estimated from plot
    x_gc =        [0  5 10 15 20 25 30 35 40 45 50 55   60 65 70 75    80   85 90  95 100];
    y_angle_deg = [12 8  4 2 0 1.75 2 3.5  6   9 15 27.5 37 20 10 10.25 10.5 11 11.25 11.5 12];

    % Interpolate & convert to radians
    y_angle_rad = deg2rad(y_angle_deg);
    theta_ref = interp1(x_gc, y_angle_rad, gc, 'pchip');

    % Out-of-range protection
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
        
        % Upper rod above ankle joint
        plot([0, 0], [0, -0.05], 'k-', 'LineWidth', 3);

        % Ankle joint
        O1 = [0; -0.05]; % Ankle joint coordinates
        plot(O1(1), O1(2), 'ko', 'MarkerFaceColor', 'w'); 
        draw_circle(O1, 0.01, 'k', 1);
        draw_circle(O1, 0.015, 'k', 1);
        
        % Lower rod below ankle joint
        plot([0, 0], [-0.05, -0.116], 'k-', 'LineWidth', 3);
        
        % GRF plane
        plot([-0.05, 0.05], [0, 0], 'b-', 'LineWidth', 3);

        % Upper rod of spring 3
        plot([-0.05, -0.05], [-0.12, 0], 'k-', 'LineWidth', 3);

        % Upper endpoint of spring 3
        plot(-0.05, -0.12, 'ko', 'MarkerFaceColor', 'm');

        foot_pts = [-0.05, -0.17; 0.19, -0.17]';
        rot_a = [cos(theta_a), -sin(theta_a); sin(theta_a), cos(theta_a)];
        foot_rot = rot_a * (foot_pts - O1) + O1;
        plot(foot_rot(1,:), foot_rot(2,:), 'b-', 'LineWidth', 3);

        % Left endpoint of foot plate (initial)
        P0 = [-0.05; -0.17];
        % Right endpoint of foot plate (initial)
        P1 = [0.19; -0.17];
        dir = (P1 - P0) / norm(P1 - P0);
        P_local = P0 + dir * 0.15;
        P_global = rot_a * (P_local - O1) + O1;
        
        % Right endpoint of spring 1
        P_f_x = P_global(1) - 0.054*sin(theta_a);
        P_f_y = P_global(2) + 0.054*cos(theta_a);
        P_f = [P_f_x,P_f_y];
        
        % Yellow pivot — left endpoint of coupling
        P_c = [0.00,-0.116];
        v = P_f - P_c;
        theta_b = atan2(v(2), v(1));
        
        % Left endpoint of spring 1 — right endpoint of leadscrew
        P_m_x = P_c(1) + (x_m+0.01)*cos(theta_b);
        P_m_y = P_c(2) + (x_m+0.01)*sin(theta_b);
        P_m = [P_m_x,P_m_y];

        plot([P_m(1), P_f(1)], [P_m(2), P_f(2)], 'r--', 'LineWidth', 2);
        
        % Right endpoint of coupling — left endpoint of leadscrew
        P_c_x = P_c(1) + 0.01*cos(theta_b);
        P_c_y = P_c(2) + 0.01*sin(theta_b);
        P_c_r = [P_c_x,P_c_y];
    
        plot([P_c_r(1), P_m(1)], [P_c_r(2), P_m(2)], 'c-', 'LineWidth', 3);
        % plot([P_m(1), P_f(1)], [P_m(2), P_f(2)], 'c-', 'LineWidth', 1);

        plot([0, P_c_r(1)], [-0.116, P_c_r(2)], 'g-', 'LineWidth', 3);
        plot(0.00, -0.116, 'ko', 'MarkerFaceColor', 'y');

        plot([P_global(1), P_f(1)], [P_global(2), P_f(2)], 'k-', 'LineWidth', 3);
        
        % Heel spring
        P_top = [-0.05; -0.12];
        local_s3 = [-0.05; -0.12];

        P_bot = rot_a * local_s3 + O1;
        P_bottom_x = P_bot(1) - 0.01*sin(theta_a);
        P_bottom_y = P_bot(2) + 0.01*cos(theta_a);
        P_bottom = [P_bottom_x,P_bottom_y];
        plot([P_top(1), P_bottom(1)], [P_top(2), P_bottom(2)], 'r--', 'LineWidth', 2);
        
        plot([P_bot(1), P_bottom(1)], [P_bot(2), P_bottom(2)], 'k-', 'LineWidth', 3);
        plot(P_bottom(1), P_bottom(2), 'ko', 'MarkerFaceColor', 'm');
        
        % Toe joint
        O2_local = [0.19; -0.17];
        O2 = rot_a * (O2_local - O1) + O1;
        draw_circle(O2, 0.01, 'k', 1);

        toe_length = 0.05;
        rot_t = [cos(theta_t), -sin(theta_t); sin(theta_t), cos(theta_t)];
        toe_vec = rot_t * [toe_length; 0];
        toe_end = O2 + toe_vec;

        plot([O2(1), toe_end(1)], [O2(2), toe_end(2)], 'b-', 'LineWidth', 3);
        plot(O2(1), O2(2), 'ko', 'MarkerFaceColor', 'w');
        draw_circle(O2, 0.006, 'r', 1);

        % Ankle bracket
        local_s4 = [-0.015; -0.12];
        P_p = rot_a * local_s4 + O1;
        P_p_x = P_p(1) - 0.12*sin(theta_a);
        P_p_y = P_p(2) + 0.12*cos(theta_a);
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
        local_s7 = [0.08; -0.12];
        P_pppp = rot_a * local_s7 + O1;
        plot([P_pppp(1), P_p_bottom_3(1)], [P_pppp(2), P_p_bottom_3(2)], 'k-', 'LineWidth', 3);

        axis equal;
        axis([-0.1 0.3 -0.2 0.05]);
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

% The heel friction torque can likely be ignored
function tau_GRF = ground_reaction_torque(t)
    % t: input time (s), scalar or vector

    % --- Simulation parameters ---
    feet_length = 0.25;           % Foot length (m)
    Body_weight = 60;             % Body weight (kg)
    T_stance = 0.5;               % Stance phase duration (s)

    % --- Original r_x data (assumed unit: % foot length) ---
    r_x = [10 15 22.5 27.5 30 32.5 37.5 42 45 51 56 60 65 70 72.5 74.5 76]; % 17 points
    x_old = linspace(0, T_stance, length(r_x));   % Corresponds to stance 0~0.5 s

    % --- Interpolate to 10 time points (or more) ---
    t_interp = linspace(0, T_stance, 10);
    r_x_interp = interp1(x_old, r_x, t_interp, 'linear');
    r_x_net = r_x_interp - 25;                    % Centered at ankle (25%)
    r_x_actual = r_x_net * feet_length / 100;     % Convert to meters

    % --- Simulated vertical GRF (as body-weight ratio) ---
    F_y = [0 9 12 11.5 10.5 9.5 9 9.5 10.5 12 11]; % 11 points
    F_y_interp = interp1(linspace(0, T_stance, length(F_y)), F_y, t_interp);
    F_y_actual = F_y_interp * Body_weight;

    % --- Torque vector for first 0.5 s ---
    tau_half = r_x_actual .* F_y_actual;  % 2D cross product: r × F (scalar)

    % --- Output tau at corresponding times ---
    tau_GRF = zeros(size(t));             % Initialize

    % Interpolate for t in [0, 0.5]
    idx = t <= T_stance;
    tau_GRF(idx) = interp1(t_interp, tau_half, t(idx), 'linear', 0);  % 0 out of range
end

function tau_toe_GRF = toe_ground_reaction_torque(t)
    % t: input time (s), scalar or vector

    % --- Simulation parameters ---
    feet_length = 0.25;           % Foot length (m)
    Body_weight = 60;             % Body weight (kg)
    T_start = 0.5;                % Toe support start time
    T_stance = 0.1;               % Toe support duration (s)

    % --- Time nodes & interpolation times ---
    t_interp = linspace(T_start, T_start + T_stance, 10);  % 0.5–0.6 s

    % --- Original r_x data (% foot length) ---
    r_x = [79 83 91];  % Force point moves toward toe
    t_r = linspace(T_start, T_start + T_stance, length(r_x));
    r_x_interp = interp1(t_r, r_x, t_interp, 'linear');

    % --- Relative to ankle at 25% ---
    r_x_net = r_x_interp - 75;
    r_x_actual = r_x_net * feet_length / 100;  % meters

    % --- Vertical GRF (BW ratio) ---
    F_y = [9 5 1];
    t_f = linspace(T_start, T_start + T_stance, length(F_y));
    F_y_interp = interp1(t_f, F_y, t_interp, 'linear');
    F_y_actual = F_y_interp * Body_weight;

    % --- Torque: tau = r × F (2D scalar) ---
    tau_toe_half = r_x_actual .* F_y_actual;

    % --- Output ---
    tau_toe_GRF = zeros(size(t));  % Initialize

    % Indices within toe support period
    idx = (t >= T_start) & (t <= T_start + T_stance);

    % Interpolate
    tau_toe_GRF(idx) = interp1(t_interp, tau_toe_half, t(idx), 'linear', 0);
end

function tau_leg_GRF = leg_ground_reaction_torque(t)
    % t: input time (s), scalar or vector

    % --- Simulation parameters ---
    Body_weight = 60;  % Body weight (kg)
    T_stance = 1;      % Stance duration (s)

    % --- Time grid for interpolation ---
    t_interp = linspace(0, T_stance, 20);

    % r_x as fraction of foot length (negative = behind ankle)
    r_x = [-5  -2.5   0    1    2      4   6   8   10   12  13 14  15 0 0 0 0 0 0 0 0]/100;
    F_y = [0     9    12  11.5 10.5   9.5  9   9.5 10.5 12  11  6  1  0 0 0 0 0 0 0 0]; % 21 points
    r_x_interp = interp1(linspace(0, T_stance, length(r_x)), r_x, t_interp);

    % Vertical GRF (BW ratio)
    F_y_interp = interp1(linspace(0, T_stance, length(F_y)), F_y, t_interp);
    F_y_actual = F_y_interp * Body_weight;

    tau_leg_half = r_x_interp .* F_y_actual;  % 2D r × F (scalar)

    % --- Output tau over time ---
    tau_leg_GRF = zeros(size(t));  % Initialize

    idx = t <= T_stance;
    tau_leg_GRF(idx) = interp1(t_interp, tau_leg_half, t(idx), 'linear', 0);
end

% Gait 50%–60%: toe push-off — reaction torque of the toe on the footplate
function toe_ankle = toe_ankle_ground_reaction_torque(t)
    % t: input time (s), scalar or vector

    % --- Simulation parameters ---
    T_start = 0.5; 
    T_stance = 0.1;             

    % Time vector corresponding to M_toe_ankle points
    t_interp = linspace(0, T_stance, length([70 40 20]));
    M_toe_ankle = [70 40 20];

    toe_ankle = zeros(size(t));   % Initialize
    
    % Indices within the support interval
    idx = (t >= T_start) & (t <= T_start + T_stance);
    
    % Shift to local time for interpolation
    t_local = t(idx) - T_start;
    
    % Interpolation
    toe_ankle(idx) = interp1(t_interp, M_toe_ankle, t_local, 'linear', 0);
end

function tau = pyStyleCross2D(a, b)
    % Input: a = [x1,y1], b = [x2,y2]
    % Output: equivalent to np.cross([x1,y1,0], [x2,y2,0])[2]
    cross_3d = cross([a(1), a(2), 0], [b(1), b(2), 0]);
    tau = cross_3d(3); % Z component
end

% % Example:
% % Time vector (0 to 1 s, 100 points)
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

