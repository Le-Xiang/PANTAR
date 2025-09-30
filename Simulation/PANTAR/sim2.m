function prosthetic_foot_sim_v4()
    % Parameter definitions
    I_a = 0.01;      % Foot plate moment of inertia (kg·m^2)
    I_t = 0.005;     % Toe joint moment of inertia (kg·m^2)
    k1 = 1000;       % Spring s1 stiffness (N/m)
    L0 = 0.04;       % Spring s1 rest length (m)
    n = 3;           % Gear ratio
    p = 0.005;       % Ball screw lead (m/rev)
    
    k3 = 1000;       % Spring s3 stiffness (N/m)
    L3 = 0.12;       % Spring s3 rest length (m)

    % Initial state: [theta_a; omega_a; x_m; v_m; theta_t; omega_t]
    y0 = [0; 0; 0.04; 0; 0; 0];

    % Simulation time range
    tspan = [0 5];

    % Solve ODE
    [t, y] = ode45(@(t,y) dynamics_v4(t, y, I_a, I_t, k1, L0, n, p, k3, L3), tspan, y0);

    % Plot ankle and toe joint angles
    figure;
    plot(t, y(:,1)*180/pi, 'LineWidth', 2);
    hold on;
    plot(t, y(:,5)*180/pi, 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    title('Ankle and toe joint angles over time');
    legend('Ankle joint', 'Toe joint');
    grid on;

    % Animation visualization
    visualize_motion_v4(t, y);
end

function dydt = dynamics_v4(t, y, I_a, I_t, k1, L0, n, p, k3, L3)
    theta_a = y(1);
    omega_a = y(2);
    x_m = y(3);
    theta_t = y(5);
    omega_t = y(6);

    % Motor input (angular velocity): ankle joint motor
    v1 = 2 * sin(2*pi*0.5*t); % Motor angular speed rad/s
    v2 = n * v1;              % Screw rotational speed
    v_m = p * v2;             % Nut linear velocity

    % Ankle joint related calculations (same as before)
    O1 = [0; -0.06];
    P_m = [x_m; -0.09];
    rot_a = [cos(theta_a), -sin(theta_a); sin(theta_a), cos(theta_a)];

    % Spring s1 torque
    local_s1 = [0.08; -0.03];
    P_f = rot_a * local_s1 + O1;
    spring_vec = P_f - P_m;
    spring_len = norm(spring_vec);
    spring_dir = spring_vec / spring_len;
    F_spring1 = k1 * (spring_len - L0) * spring_dir;
    r1 = P_f - O1;
    torque1 = cross2D(r1, F_spring1);

    % Spring s3 torque (dynamic)
    P_top = [-0.03; 0];
    local_s3 = [-0.03; -0.06];
    P_bottom = rot_a * local_s3 + O1;
    spring_vec3 = P_top - P_bottom;
    spring_len3 = norm(spring_vec3);
    spring_dir3 = spring_vec3 / spring_len3;
    F_spring3 = k3 * (spring_len3 - L3) * spring_dir3;
    r3 = P_bottom - O1;
    torque3 = cross2D(r3, F_spring3);

    total_torque_a = torque1 + torque3;

    % Toe joint motor torque input (N·m), example sine wave
    tau_motor_t = 0.01 * sin(2*pi*0.5*t);

    % State derivatives
    dydt = zeros(6,1);
    dydt(1) = omega_a;
    dydt(2) = total_torque_a / I_a;
    dydt(3) = v_m;
    dydt(4) = 0; % Nut acceleration = 0
    dydt(5) = omega_t;
    dydt(6) = tau_motor_t / I_t;
end

function z = cross2D(a, b)
    z = a(1)*b(2) - a(2)*b(1);
end

function visualize_motion_v4(t, y)
    L1 = 0.06; % Upper rod length
    for i = 1:10:length(t)
        theta_a = y(i,1);
        x_m = y(i,3);
        theta_t = y(i,5);

        clf; hold on;

        % Upper rod U to O1
        plot([0, 0], [0, -0.06], 'k-', 'LineWidth', 3);
        O1 = [0; -0.06];
        plot(O1(1), O1(2), 'ko', 'MarkerFaceColor', 'k');
        draw_circle(O1, 0.01, 'k', 1);

        % Support plane F
        plot([-0.03, 0.03], [0, 0], 'g-', 'LineWidth', 3);

        % Foot plate from [-0.03, -0.12] to [0.16, -0.12], rotated by ankle angle
        foot_pts = [-0.03, -0.12; 0.16, -0.12]';
        rot_a = [cos(theta_a), -sin(theta_a); sin(theta_a), cos(theta_a)];
        foot_rot = rot_a * (foot_pts - O1) + O1;
        plot(foot_rot(1,:), foot_rot(2,:), 'b-', 'LineWidth', 5);

        % Spring s1
        P_m = [x_m; -0.09];
        local_s1 = [0.08; -0.03];
        P_f = rot_a * local_s1 + O1;
        plot([P_m(1), P_f(1)], [P_m(2), P_f(2)], 'r--', 'LineWidth', 2);

        % Spring s3
        P_top = [-0.03; 0];
        local_s3 = [-0.03; -0.06];
        P_bottom = rot_a * local_s3 + O1;
        plot([P_top(1), P_bottom(1)], [P_top(2), P_bottom(2)], 'm--', 'LineWidth', 2);

        % Toe joint O2
        O2_local = [0.16; -0.12]; % Toe joint center initial point
        O2 = rot_a * (O2_local - O1) + O1; % Position affected by ankle rotation

        % Draw toe joint circle (radius 0.01 m)
        viscircles(O2', 0.01, 'Color', 'k', 'LineWidth', 1, 'LineStyle', '--');
        % draw_circle(O2, 0.01, 'k', 1);

        % Toe bar extends 0.05 m to the right
        toe_length = 0.05; 
        rot_t = [cos(theta_t), -sin(theta_t); sin(theta_t), cos(theta_t)];
        toe_vec = rot_t * [toe_length; 0];
        toe_end = O2 + toe_vec;

        % Draw toe bar
        plot([O2(1), toe_end(1)], [O2(2), toe_end(2)], 'm-', 'LineWidth', 4);

        axis equal;
        axis([-0.1 0.3 -0.15 0.05]);
        title(sprintf('Time %.2f s', t(i)));
        xlabel('X (m)'); ylabel('Y (m)');
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
