% Non-minimal version with PID
function prosthetic_foot_sim_v7()
    % Parameter definitions
    I_a = 0.01;       % Inertia of footplate (kg·m^2)
    I_t = 0.0001;     % Inertia of toe joint (kg·m^2)
    m_t = 0.08;       % Toe mass (kg)
    d = 0.16;         % Distance from toe joint to ankle joint (m)
    
    % Additional inertia of toe mass seen at ankle (parallel axis theorem)
    I_toe_at_ankle = I_t + m_t * d^2;
    I_a_total = I_a + I_toe_at_ankle;

    % Spring parameters
    k1 = 1000; L0 = 0.04;
    n = 3; p = 0.005;
    k3 = 1000; L3 = 0.12;
    n_t = 5; theta_t0 = 0;

    % Damping parameters
    b_a = 0.05;       % Ankle damping N·m·s/rad
    b_t = 0.01;       % Toe joint damping N·m·s/rad
    b_m = 0.005;      % Motor shaft damping (toe joint) N·m·s/rad

    % Initial states:
    % Added 4 new states: integral errors and previous errors for ankle and toe joints
    % [ankle angle; ankle velocity; motor displacement; motor velocity; 
    %  toe angle; toe velocity; ankle integral error; toe integral error]
    y0 = [0; 0; 0.04; 0; 0; 0; 0; 0]; 

    tspan = [0 5];

    [t, y] = ode45(@(t,y) dynamics_v7(t, y, ...
        I_a_total, I_t, k1, L0, n, p, k3, L3, ...
        n_t, theta_t0, b_a, b_t, b_m), tspan, y0);

    % Plot: ankle angle tracking
    figure();
    plot(t, y(:,1)*180/pi, 'LineWidth', 2); hold on;
    plot(t, arrayfun(@ankle_trajectory, t)*180/pi, '--r', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    title('Ankle Joint Angle Tracking (solid = actual, dashed = reference)');
    legend('Actual', 'Reference');
    grid on;

    % Plot: toe joint angle tracking
    figure();
    plot(t, y(:,5)*180/pi, 'LineWidth', 2); hold on;
    plot(t, arrayfun(@toe_trajectory, t)*180/pi, '--r', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    title('Toe Joint Angle Tracking (solid = actual, dashed = reference)');
    legend('Actual', 'Reference');
    grid on;
    
    % Motion visualization
    figure();
    visualize_motion_v7(t, y);
end

function dydt = dynamics_v7(t, y, I_a_total, I_t, ...
    k1, L0, n, p, k3, L3, n_t, theta_t0, b_a, b_t, b_m)

    theta_a = y(1); omega_a = y(2);
    x_m = y(3); omega_m = y(4);  % Motor angular velocity
    theta_t = y(5); omega_t = y(6);
    integral_e_a = y(7);
    integral_e_t = y(8);

    % Reference trajectories
    theta_a_ref = ankle_trajectory(t);
    theta_t_ref = toe_trajectory(t);

    dt = 0.01; % Sampling time for PID integration/differentiation
    % --- Ankle PID: compute motor velocity reference ---
    e_a = theta_a_ref - theta_a;
    integral_e_a = integral_e_a + e_a * dt;

    Kp_a = 10; Ki_a = 1; Kd_a = 40;
    % omega_m_ref = Kp_a*e_a + Ki_a*integral_e_a + Kd_a*de_a;
    omega_m_ref = Kp_a*e_a + Ki_a*integral_e_a;

    % --- Motor speed loop: compute motor angular acceleration ---
    K_motor = 5; % Motor speed loop gain
    alpha_m = K_motor * (omega_m_ref - omega_m);

    % --- Toe joint PID ---
    e_t = theta_t_ref - theta_t;
    integral_e_t = integral_e_t + e_t * dt;

    Kp_t = 5; Ki_t = 1; Kd_t = 40;
    % tau_pid_t = Kp_t*e_t + Ki_t*integral_e_t + Kd_t*de_t;
    tau_pid_t = Kp_t*e_t + Ki_t*integral_e_t;

    % --- Spring & damping forces ---
    O1 = [0; -0.06]; P_m = [x_m; -0.09];
    rot_a = [cos(theta_a), -sin(theta_a); sin(theta_a), cos(theta_a)];
    local_s1 = [0.08; -0.03];
    P_f = rot_a * local_s1 + O1;
    spring_vec = P_f - P_m;
    spring_len = norm(spring_vec);
    spring_dir = spring_vec / spring_len;
    F_spring1 = k1 * (spring_len - L0) * spring_dir;
    r1 = P_f - O1;
    torque1 = cross2D(r1, F_spring1);

    P_top = [-0.03; 0];
    local_s3 = [-0.03; -0.06];
    P_bottom = rot_a * local_s3 + O1;
    spring_vec3 = P_top - P_bottom;
    spring_len3 = norm(spring_vec3);
    spring_dir3 = spring_vec3 / spring_len3;
    F_spring3 = k3 * (spring_len3 - L3) * spring_dir3;
    r3 = P_bottom - O1;
    torque3 = cross2D(r3, F_spring3);

    % Ankle total torque: spring + damping (+ optional GRF)
    tau_GRF = ground_reaction_torque(t);
    total_torque_a = torque1 - b_a * omega_a;
    % Motor torque is represented indirectly by motor speed loop, not added here

    % Toe joint torque: PID + spring + damping + GRF
    torque_spring_t = -0.2 * (theta_t - theta_t0);
    damping_joint_t = -b_t * omega_t;
    damping_motor_t = - (b_m / n_t^2) * omega_t;
    tau_toe_GRF = toe_ground_reaction_torque(t);
    total_torque_t = tau_pid_t / n_t + torque_spring_t + damping_joint_t + damping_motor_t + tau_toe_GRF;

    % State derivatives
    dydt = zeros(8,1);
    dydt(1) = omega_a;
    dydt(2) = total_torque_a / I_a_total;
    dydt(3) = omega_m * n * p;    % Motor screw displacement (lead p)
    dydt(4) = alpha_m;            % Motor angular acceleration
    dydt(5) = omega_t;
    dydt(6) = total_torque_t / I_t;
    dydt(7) = e_a;
    dydt(8) = e_t;
end

function theta_ref = ankle_trajectory(t)
    theta_start = -20 * pi/180;
    theta_end = 10 * pi/180;
    T = 5;
    if t <= T
        theta_ref = theta_start + (theta_end - theta_start) * t / T;
    else
        theta_ref = theta_end;
    end
end

function theta_ref = toe_trajectory(t)
    theta_start = -10 * pi/180;
    theta_end = 50 * pi/180;
    T = 5;
    if t <= T
        theta_ref = theta_start + (theta_end - theta_start) * t / T;
    else
        theta_ref = theta_end;
    end
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

        % Shank
        plot([0, 0], [0, -0.06], 'k-', 'LineWidth', 3);
        O1 = [0; -0.06];
        plot(O1(1), O1(2), 'ko', 'MarkerFaceColor', 'w');
        draw_circle(O1, 0.01, 'k', 1);

        plot([0, 0], [-0.06, -0.09], 'k-', 'LineWidth', 3);

        % Socket adapter
        plot([-0.03, 0.03], [0, 0], 'b-', 'LineWidth', 3);

        % Footplate
        foot_pts = [-0.03, -0.12; 0.16, -0.12]';
        rot_a = [cos(theta_a), -sin(theta_a); sin(theta_a), cos(theta_a)];
        foot_rot = rot_a * (foot_pts - O1) + O1;
        plot(foot_rot(1,:), foot_rot(2,:), 'b-', 'LineWidth', 5);

        % Series spring
        P_m = [x_m; -0.09];
        local_s1 = [0.08; -0.03];
        P_f = rot_a * local_s1 + O1;
        plot([P_m(1), P_f(1)], [P_m(2), P_f(2)], 'r--', 'LineWidth', 2);

        plot([0, P_m(1)], [P_m(2), -0.09], 'k--', 'LineWidth', 3);

        % Arch support linkage
        P0 = [-0.03; -0.12];
        P1 = [0.16; -0.12];
        dir = (P1 - P0) / norm(P1 - P0);
        P_local = P0 + dir * 0.11;
        P_global = rot_a * (P_local - O1) + O1;
        plot([P_global(1), P_f(1)], [P_global(2), P_f(2)], 'k-', 'LineWidth', 3);

        % Parallel spring
        P_top = [-0.03; 0];
        local_s3 = [-0.03; -0.06];
        P_bottom = rot_a * local_s3 + O1;
        plot([P_top(1), P_bottom(1)], [P_top(2), P_bottom(2)], 'r--', 'LineWidth', 2);

        % Toe joint
        O2_local = [0.16; -0.12];
        O2 = rot_a * (O2_local - O1) + O1;
        draw_circle(O2, 0.01, 'k', 1);

        toe_length = 0.05;
        rot_t = [cos(theta_t), -sin(theta_t); sin(theta_t), cos(theta_t)];
        toe_vec = rot_t * [toe_length; 0];
        toe_end = O2 + toe_vec;

        plot([O2(1), toe_end(1)], [O2(2), toe_end(2)], 'b-', 'LineWidth', 4);
        plot(O2(1), O2(2), 'ko', 'MarkerFaceColor', 'w');
        draw_circle(O2, 0.005, 'r', 1);

        axis equal;
        axis([-0.1 0.3 -0.15 0.05]);
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

% Ground reaction torque at ankle (simplified model)
function tau_GRF = ground_reaction_torque(t)
    T = 1;             % Gait cycle duration (s)
    F_max = 600;       % Maximum vertical GRF (N)
    % Vertical GRF approximated as sine wave
    F_y = max(0, F_max * sin(pi * mod(t, T) / T));
    
    % Center of pressure moves from heel (-0.05 m) to toe (+0.05 m)
    r_x = 0.05 * sin(2 * pi * mod(t, T) / T)+0.02;
    
    % Torque = moment arm × force
    tau_GRF = r_x * F_y;
end

% Ground reaction torque at toe
function tau_toe_GRF = toe_ground_reaction_torque(t)
    T = 1;              % Gait cycle duration (s)
    F_toe_max = 100;    % Maximum toe vertical load (N)
    mu = 0.5;           % Friction coefficient

    % Vertical force
    F_v = max(0, F_toe_max * sin(pi * mod(t, T) / T));

    % Frictional force (simplified model)
    F_f = - mu * F_v; 

    % Moment arm (toe joint to contact point, m)
    r_toe = 0.03 * sin(2 * pi * mod(t, T) / T)+0.03; 

    % Torque = (vertical + frictional force) × moment arm
    tau_toe_GRF = r_toe * (F_v + F_f);
end
