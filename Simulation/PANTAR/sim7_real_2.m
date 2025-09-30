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

    % Parameter definitions:
    % The part above the ankle joint that stays fixed may be <1 kg; the toe part ~0.1 kg;
    % overall ~3 kg is reasonable (context note from original).
    I_a = 0.01;      % Footplate moment of inertia (kg·m^2); roughly 2 kg plus materials → ~2.5 kg
    I_t = 0.0001;    % Toe joint moment of inertia (kg·m^2)
    m_t = 0.1;       % Toe mass (kg)
    d = 0.20;        % Distance from toe joint to ankle joint (m)
    eta = 0.9;       % Efficiency factor η 
    
    % There are other inertias to add (motors and parts on the plate) not yet included.
    % (This applies to swing + stance, excluding the sub-phase from full-foot contact to forefoot
    % contact up to toe-off.)
    I_toe_at_ankle = I_t + m_t * d^2;
    I_a_total = I_a + I_toe_at_ankle;
    
    % In the excluded process, ankle rotation doesn’t need to consider toe rotation.
    % I_a_total = I_a;
    
    % System 1 - Ankle joint
    k1_1 = 400000; % Spring k1 (plantarflexion) stiffness
    k1_2 = 400000; % Spring k1 (dorsiflexion) stiffness
    L0 = 0.06;     % Spring k1 free length (was 0.0635)
    n = 1;         % Gear ratio for k1 system
    p_m_per_rev = 0.01; % Leadscrew pitch (m/rev). Larger pitch → less torque capacity, smaller → slower response.
                        % Too large may overshoot. Final range used here: 0.01 m (10 mm).
    p = p_m_per_rev / (2 * pi);  % m/rad
    theta_a0 = deg2rad(-5);      % Initial ankle angle

    % System 3 - Ankle joint
    k3 = 0;        % Spring k3 stiffness
    L3 = 0.03;     % Spring k3 free length (was 0.0485)
    
    % System 2 - Toe joint
    n_t = 0.5;               % Gear ratio for k2 system
    theta_t0 = deg2rad(12);  % Initial toe angle
    I_actual = 0;            % Actual motor current (external input)
    k2 = 1.075;              % Spring k2 stiffness

    % Damping parameters
    b_a = 0.05;   % Ankle damping N·m·s/rad
    b_t = 0.01;   % Toe joint damping N·m·s/rad
    b_m = 0.005;  % Motor shaft (toe joint) damping N·m·s/rad

    % Initial state:
    % Add 4 error-integral/previous-error states: ankle integral error, previous ankle error,
    % toe integral error, previous toe error
    y0 = [theta_a0; 0; 0.01; 0; theta_t0; 0; 0; 0]; 

    tspan = [0 0.0001];
    % tspan = linspace(0, 1, 1000);
    % options = odeset('RelTol', 1e-8, 'AbsTol', 1e-8);  % increase accuracy

    [t, y] = ode45(@(t,y) dynamics_v7(t, y, ...
        I_a_total, I_t, k1_1, k1_2, L0, n, p, k3, L3, ...
        n_t, b_a, b_t, b_m, eta, I_actual, k2), tspan, y0);

    % [t, y] = ode45(@(t,y) dynamics_v7(t, y, ...
    %     I_a_total, I_t, k1, L0, n, p, k3, L3, ...
    %     n_t, b_a, b_t, b_m, eta, I_actual, k2), tspan, y0, options);

    % Plot: ankle angle tracking
    figure();
    plot(t, y(:,1)*180/pi, 'LineWidth', 2); hold on;
    plot(t, arrayfun(@ankle_trajectory_real, t)*180/pi, '--r', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    title('Ankle angle tracking (solid = actual, dashed = desired)');
    legend('Actual angle', 'Desired trajectory');
    grid on;

    % Plot: toe angle tracking
    figure();
    plot(t, y(:,5)*180/pi, 'LineWidth', 2); hold on;
    plot(t, arrayfun(@toe_trajectory_real, t)*180/pi, '--r', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    title('Toe angle tracking (solid = actual, dashed = desired)');
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
    k1_1,k1_2, L0, n, p, k3, L3, n_t, b_a, b_t, b_m, eta, I_actual, k2)

    theta_a = y(1); % Ankle angle
    omega_a = y(2); % Ankle angular velocity
    x_m = y(3);     % Ballscrew nut position
    omega_m = y(4); % Motor angular velocity
    theta_t = y(5); % Toe angle
    omega_t = y(6); % Toe angular velocity
    integral_e_a = y(7); 
    integral_e_t = y(8);

    % Desired trajectories
    theta_a_ref = ankle_trajectory_real(t);
    theta_t_ref = toe_trajectory_real(t);
    omega_a_ref = ankle_angular_velocity_real(t);

    % --- Ankle PID: compute desired motor speed ---
    e_a = theta_a_ref - theta_a;
    % Typically omega_a_ref = 0 for point regulation
    de_a = omega_a_ref - omega_a;  % Use speed error as error derivative approximation

    Kp_a = 800; Ki_a = 10; Kd_a = 20;
    % max_val = 10;  % anti-windup example
    % integral_e_a = max(min(integral_e_a, max_val), -max_val);

    omega_m_ref = (Kp_a*e_a + Ki_a*integral_e_a + Kd_a * de_a);
    % omega_m_ref = (Kp_a*e_a);
    fprintf('t=%.2f, e_a=%.4f, int_e_a=%.4f, Ki*int=%.4f\n', ...
    t, e_a, integral_e_a, Ki_a*integral_e_a);

    fprintf('t=%.3f, omega_m_ref=%.4f', t, omega_m_ref);
    fprintf('t=%.3f, x_m=%.4f,e_a=%.4f', t, x_m, e_a);

    % --- Motor speed loop: compute motor angular acceleration ---
    K_motor = 1; % Motor speed-loop gain
    alpha_m = K_motor * (omega_m_ref - omega_m);

    % Alternative: full PID control

    % --- Toe joint PID ---
    e_t = theta_t_ref - theta_t;
    Kp_t = 800; Ki_t = 20;
    tau_pid_t = Kp_t*e_t + Ki_t*integral_e_t;

    % Torque → target current
    k_t = 0.5;
    I_ref = tau_pid_t / k_t;
    % Inner controller: current PI with sensor feedback (placeholder)
    % error_I = I_ref - I_actual;
    % voltage_out = PI_current_control(error_I);
    % set_motor_voltage(voltage_out);

    % Geometry/length parameters (meters)
    d_r = 0.05;
    d_d = 0.005;   % Heel distance from left edge of receiving plane
    l_k = 20/100;  % Footplate length (no toe)
    l_t = 7/100;   % Toe length
    h_a = 7/100;   % Ankle height (from ground plane)
    h_m = 1/100;   % Height of transmission pivot (from ground plane)
    h_t = 2/100;   % Toe joint height (from ground plane)
    h_f = 21/100;  % Total foot height (from ground plane)
    d_a = 6/100;   % Horizontal ankle position from heel
    l_k3 = 4.85/100; % Heel spring k3 free length (compression spring)
    d_k1 = 10/100; % Spring k1 left anchor position along footplate from heel
    l_k1 = 6/100;  % Spring k1 free length (compression spring)
    h_s = 14/100;  % Vertical distance ankle ↔ receiving plane
    h_k3 = 4/100;  % Vertical offset from footplate to k3 top
    d_k3 = 1/100;  % Horizontal distance of k3 to heel
    t_k1 = 1.5/100;% Vertical distance from k1 left endpoint to footplate
    h_c = -5/100;  % Vertical offset between transmission pivot and coupler
    l_c = 1/100;   % Coupler equivalent length
    l_g1 = 2.5/100;% Leadscrew device horizontal length (left)
    l_g2 = 5/100;  % Leadscrew device horizontal length (right)
    limit = 3/100; % Leadscrew stroke limit
    
    % Ankle joint
    O1 = [0; -h_s]; % Ankle joint coordinates

    % Footplate
    foot_pts = [-d_r-d_d, -h_f; -d_r-d_d+l_k, -h_f]';
    rot_a = [cos(theta_a), -sin(theta_a); sin(theta_a), cos(theta_a)];
    foot_rot = rot_a * (foot_pts - O1) + O1;

    % Original left footplate endpoint
    P0 = [-d_r-d_d; -h_f];

    % Original right footplate endpoint
    P1 = [-d_r-d_d+l_k; -h_f];
    dir = (P1 - P0) / norm(P1 - P0);
    P_local = P0 + dir * d_k1;
    % Spring k1 left endpoint position on footplate
    P_global = rot_a * (P_local - O1) + O1;
    
    % Ankle bar position on footplate
    P_local2 = P0 + dir * (d_a-d_d);
    P_global2 = rot_a * (P_local2 - O1) + O1;
    
    % Spring k1 left endpoint (fixed to plate)
    P_f_x = P_global(1) - t_k1*sin(theta_a);
    P_f_y = P_global(2) + t_k1*cos(theta_a);
    P_f = [P_f_x; P_f_y];
    
    % Yellow pivot: transmission rotation point
    P_t = [0.00;-(h_f-h_m)];

    % Coupler equivalent endpoints (left, right = screw left)
    P_c_o = [0.00,-(h_f-h_m)-h_c;0.00+l_c,-(h_f-h_m)-h_c]';
    P_c = rot_a * (P_c_o - P_t) + P_t;

    % Leadscrew right endpoint (fixed part, left side)
    P_m_x = P_c(1,2) + l_g1*cos(theta_a);
    P_m_y = P_c(2,2) + l_g1*sin(theta_a);
    P_m1 = [P_m_x;P_m_y];

    % Leadscrew nut left position (variable part)
    P_m_x = P_c(1,2) + (x_m+l_g1)*cos(theta_a);
    P_m_y = P_c(2,2) + (x_m+l_g1)*sin(theta_a);
    P_m2 = [P_m_x;P_m_y];

    % Nut seat left end → spring k1 (plantarflexion) right endpoint
    P_m_x = P_c(1,2) + (x_m+l_g1+l_g2)*cos(theta_a);
    P_m_y = P_c(2,2) + (x_m+l_g1+l_g2)*sin(theta_a);
    P_m3 = [P_m_x;P_m_y];

    % Nut seat right end → spring k1 (dorsiflexion) left endpoint
    P_m_x = P_c(1,2) + (x_m+l_g1+l_g2+l_g3)*cos(theta_a);
    P_m_y = P_c(2,2) + (x_m+l_g1+l_g2+l_g3)*sin(theta_a);
    P_m4 = [P_m_x;P_m_y];
 
    % Rod between screw right end and spring right endpoint (geometric helper)
    P_z_x = P_m3(1) + (h_m-h_c-t_k1)*sin(theta_a);
    P_z_y = P_m3(2) - (h_m-h_c-t_k1)*cos(theta_a);
    P_z = [P_z_x;P_z_y];

    % k1 torque on ankle (plantarflexion spring path shown)
    spring_vec = P_z - P_f;
    spring_len = norm(spring_vec);
    spring_dir = spring_vec / spring_len;
    F_spring1 = k1_1 * (l_k1 - spring_len) * spring_dir;
    F1 = norm(F_spring1);
    r1 = P_f - O1; % Moment arm of k1
    % Sign note: depends on coordinate convention; negative matches previous MATLAB geometry.
    torque1 = -cross2D(r1, F_spring1);
    % torque1 = pyStyleCross2D(r1, F_spring1); 

    % Heel spring
    P_top = [-d_r-d_d+d_k3; -(h_f-h_k3)];
    local_s3 = [-d_r-d_d+d_k3; -h_a];

    P_bot = rot_a * local_s3 + O1;

    % Spring k3
    spring_vec3 = P_top - P_bot;
    spring_len3 = norm(spring_vec3);
    spring_dir3 = spring_vec3 / spring_len3;
    % Note: if spring_len3 looks off, re-check geometry
    F_spring3 = k3 * (spring_len3 - l_k3) * spring_dir3;
    F3 = norm(F_spring3);
    r3 = P_bot - O1;
    torque3 = cross2D(r3, F_spring3);
    
    fprintf(['t=%.3f, torque1=%.4f, torque3=%.4f, spring_len=%.4f, spring_len3=%.4f, ', ...
             'F1=%.4f, F3=%.4f, tau_pid_t = %.4f, omega_t = %.4f\n  '], ...
            t, torque1, torque3, spring_len, spring_len3, F1, F3, tau_pid_t, omega_t);

    % Total ankle torque: PID output (via speed loop), springs, damping, support torques
    toe_ankle = toe_ankle_ground_reaction_torque(t);
    tau_GRF = ground_reaction_torque(t);
    tau_leg = leg_ground_reaction_torque(t);
    total_torque_a = torque1 + torque3 - b_a * omega_a + tau_GRF - tau_leg + toe_ankle;
    % total_torque_a = torque1 + torque3 - b_a * omega_a;
    % Note: ankle motor torque is controlled by a speed loop, not summed directly here.

    % Toe joint torque
    torque_spring_t = -k2 * (theta_t - 0);
    damping_joint_t = -b_t * omega_t;
    damping_motor_t = - (b_m / n_t^2) * omega_t;
    tau_toe_GRF = toe_ground_reaction_torque(t);
    total_torque_t = tau_pid_t / n_t + torque_spring_t + damping_joint_t + damping_motor_t + tau_toe_GRF;
    
    % Hard limits for ankle angle (rad)
    theta_a_min = deg2rad(-12.5);
    theta_a_max = deg2rad(10);
    
    % Hard limits for toe angle (rad)
    theta_t_min = deg2rad(0);
    theta_t_max = deg2rad(37);
    
    % Apply joint hard limits
    if theta_a < theta_a_min
        theta_a = theta_a_min;
        omega_a = 0;  % stop further motion (optionally add damping)
    elseif theta_a > theta_a_max
        theta_a = theta_a_max;
        omega_a = 0;
    end
    
    if theta_t < theta_t_min
        theta_t = theta_t_min;
        omega_t = 0;
    elseif theta_t > theta_t_max
        theta_t = theta_t_max;
        omega_t = 0;
    end

    % State derivatives
    dydt = zeros(8,1);
    dydt(1) = omega_a;
    dydt(2) = total_torque_a / I_a_total;

    dx_m = omega_m * n * p;  % Leadscrew position derivative; p is pitch
    fprintf('t=%.3f, dx_m=%.4f', t, dx_m);
    
    % Stroke limit (limit derivatives, not states) defined by screw travel
    x_min = 0; x_max = limit;
    if y(3) <= x_min && dx_m < 0
        dx_m = 0;    % Directly clamp derivative
        alpha_m = 0; % Also clamp motor accel
    elseif y(3) >= x_max && dx_m > 0
        dx_m = 0;
        alpha_m = 0;
    end
    
    % Optional geometric limit between x_m and P_f to avoid over-compression to zero:
    % if y(3) >= norm(v) && dx_m > 0
    %     dx_m = 0;    
    %     alpha_m = 0;
    % elseif norm(v) <= 0
    %     dx_m = 0;    
    %     alpha_m = 0;
    % end

    dydt(3) = dx_m;
    dydt(4) = alpha_m;        % Motor angular acceleration from speed loop
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
    % Simulate normal human ankle angle trajectory (black solid line)
    % Input: t ∈ [0, 1] s
    % Output: theta_ref (rad)

    % Map t ∈ [0,1] to %GC ∈ [0,100]
    gc = t * 100;

    % Data estimated from plot (deg)
    x_gc =        [0    5  10  15  20 25  30  35 40  45  50 55 60   65   70 75 80 85  90  95 100];
    y_angle_deg = [-5 -7.5 -5 -2.5 0 1.25 2.5 5  6.5 8.5 10 5  -5 -12.5 -10 -5 -1  0  -2 -3.5 -5];
                 
    % Interpolate and convert to radians
    y_angle_rad = deg2rad(y_angle_deg);
    theta_ref = interp1(x_gc, y_angle_rad, gc, 'pchip');

    % Out-of-range protection
    theta_ref(gc < 0 | gc > 100) = 0;
end

function omega_ref = ankle_angular_velocity_real(t)
    dt = 0.001;  % Differencing step; tune by control frequency
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

    % Blue curve data estimated from plot (deg)
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
    % Parameter set (meters). Coordinate system origin: center of the receiving plane.
    % Ignore the protrusion height of the receiving plane and footplate thickness (actual 1.4 cm).
    % Standard Cartesian axes: +x right, +y up.

    % Receiving plane half-lengths
    d_r = 5.8/100;
    d_d = 1.6/100;   % Heel distance from left edge of receiving plane
    l_k = 23/100;    % Footplate length (no toe)
    l_t = 7.5/100;   % Toe length
    h_a = 7.3/100;   % Ankle height (from ground)
    h_m = 3.2/100;   % Transmission pivot height (from ground)
    h_t = 1.3/100;   % Toe joint height (from ground)
    h_f = 19.5/100;  % Total foot height (from ground)
    d_a = 7.5/100;   % Horizontal ankle position from heel
    h_s = 12.2/100;  % Vertical distance ankle ↔ receiving plane
    l_k3 = 4/100;    % Heel spring k3 free length (compression spring). In this sim it’s uncertain due to sleeve+pin design.
    h_k3 = 4/100;    % Vertical offset from footplate to k3 top
    d_k3 = 4.5/100;  % Horizontal distance of k3 to heel
    h_c = 0/100;     % Vertical offset transmission pivot ↔ coupler (new design sets them coplanar → 0)
    l_c = 0/100;     % Coupler equivalent length (not used)

    % The following variables depend on design:
    d_k1_1 = 2.6/100; % Spring k1 (plantarflexion) left anchor position (from heel)
    l_k1_1 = 4/100;   % Spring k1 (plantarflexion) free length (compression)
    d_k1_2 = 11/100;  % Spring k1 (dorsiflexion) left anchor position (from heel; actually near right)
    l_k1_2 = 4/100;   % Spring k1 (dorsiflexion) free length (compression)
    t_k1 = h_m;       % Vertical distance from k1 to footplate so its action point aligns with screw & coupler

    l_g1 = 3.3/100; % Leadscrew device own horizontal length (left)
    l_g2 = 0.9/100; % Leadscrew device horizontal length (middle; nut seat offset)
    l_g3 = 0.5/100; % Leadscrew device horizontal length (right; nut seat offset)

    for i = 1:10:length(t)
        theta_a = y(i,1);
        x_m = y(i,3);
        theta_t = y(i,5);

        clf; hold on;
        
        % Upper link above ankle
        plot([0, 0], [0, -h_s], 'k-', 'LineWidth', 3);

        % Ankle joint
        O1 = [0; -h_s]; % Ankle coordinates
        plot(O1(1), O1(2), 'ko', 'MarkerFaceColor', 'w'); 
        draw_circle(O1, 0.01, 'k', 1);
        draw_circle(O1, 0.015, 'k', 1);
        
        % GRF receiving plane
        plot([-d_r, d_r], [0, 0], 'b-', 'LineWidth', 3);

        % Upper rod of spring 3
        plot([-d_r, -d_r], [-(h_f-h_k3), 0], 'k-', 'LineWidth', 3);

        % Footplate
        foot_pts = [-d_r-d_d, -h_f; -d_r-d_d+l_k, -h_f]';
        rot_a = [cos(theta_a), -sin(theta_a); sin(theta_a), cos(theta_a)];
        foot_rot = rot_a * (foot_pts - O1) + O1;
        plot(foot_rot(1,:), foot_rot(2,:), 'b-', 'LineWidth', 3);

        % Left footplate endpoint (initial)
        P0 = [-d_r-d_d; -h_f];

        % Spring k1 (plantarflexion) geometry
        P1 = [-d_r-d_d+l_k; -h_f];
        dir = (P1 - P0) / norm(P1 - P0);
        P_local1_1 = P0 + dir * d_k1_1;
        % Spring k1 plantarflexion left anchor on plate
        P_global1_1 = rot_a * (P_local1_1 - O1) + O1;

        % Spring k1 plantarflexion left anchor (fixed to plate)
        P_f1_x = P_global1_1(1) - t_k1*sin(theta_a);
        P_f1_y = P_global1_1(2) + t_k1*cos(theta_a);
        P_f1 = [P_f1_x; P_f1_y];

        % Short rod between plate and the lowered spring anchor
        plot([P_global1_1(1), P_f1(1)], [P_global1_1(2), P_f1(2)], 'k-', 'LineWidth', 3);
        
        % Spring k1 (dorsiflexion) geometry
        P1 = [-d_r-d_d+l_k; -h_f];
        dir = (P1 - P0) / norm(P1 - P0);
        P_local1_2 = P0 + dir * d_k1_2;
        % Spring k1 dorsiflexion right anchor on plate
        P_global1_2 = rot_a * (P_local1_2 - O1) + O1;

        % Spring k1 dorsiflexion right anchor (fixed to plate)
        P_f2_x = P_global1_1(1) - t_k1*sin(theta_a);
        P_f2_y = P_global1_1(2) + t_k1*cos(theta_a);
        P_f2 = [P_f2_x; P_f2_y];

        % Short rod for dorsiflexion path
        plot([P_global1_2(1), P_f2(1)], [P_global1_2(2), P_f2(2)], 'k-', 'LineWidth', 3);
        
        % Ankle bar position on plate
        P_local2 = P0 + dir * (d_a-d_d);
        P_global2 = rot_a * (P_local2 - O1) + O1;
        plot([P_global2(1), 0], [P_global2(2), -h_s], 'k-', 'LineWidth', 3);
        
        % Yellow pivot — transmission rotation point
        P_t = [0.00;-(h_f-h_m)];
        plot(0.00, -(h_f-h_m), 'ko', 'MarkerFaceColor', 'w');

        % (Coupler is omitted by design here; both pivot & coupler coplanar.)

        % Leadscrew right end segments
        % Fixed part (left)
        P_m_x = P_t(1,2) + l_g1*cos(theta_a);
        P_m_y = P_t(2,2) + l_g1*sin(theta_a);
        P_m1 = [P_m_x,P_m_y];
        
        % Variable part nut left
        P_m_x = P_t(1,2) + (x_m+l_g1)*cos(theta_a);
        P_m_y = P_t(2,2) + (x_m+l_g1)*sin(theta_a);
        P_m2 = [P_m_x,P_m_y];
        
        % Nut seat left → spring k1 plantarflexion right endpoint
        P_m_x = P_t(1,2) + (x_m+l_g1+l_g2)*cos(theta_a);
        P_m_y = P_t(2,2) + (x_m+l_g1+l_g2)*sin(theta_a);
        P_m3 = [P_m_x,P_m_y];

        % Nut seat right → spring k1 dorsiflexion left endpoint
        P_m_x = P_t(1,2) + (x_m+l_g1+l_g2+l_g3)*cos(theta_a);
        P_m_y = P_t(2,2) + (x_m+l_g1+l_g2+l_g3)*sin(theta_a);
        P_m4 = [P_m_x,P_m_y];
     
        % Visualize segments
        plot([P_c(1,2), P_m1(1)], [P_c(2,2), P_m1(2)], 'k-', 'LineWidth', 3);
        plot([P_m1(1), P_m2(1)], [P_m1(2), P_m2(2)], 'c-', 'LineWidth', 3);
        plot([P_m2(1), P_m3(1)], [P_m2(2), P_m3(2)], 'r--', 'LineWidth', 3);
        plot([P_m3(1), P_m4(1)], [P_m3(2), P_m4(2)], 'k-', 'LineWidth', 3);
        plot([P_m4(1), P_f2(1)], [P_m4(2), P_f2(2)], 'r--', 'LineWidth', 3);

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
        
        % Short link between heel spring top and its upper rod
        plot([P_top(1),P_top(1)-d_d+d_k3], [P_top(2), P_top(2)], 'k-', 'LineWidth', 3);
        
        plot(P_bot(1), P_bot(2), 'ko', 'MarkerFaceColor', 'm');
        plot(P_top(1)-d_d+d_k3, P_top(2), 'ko', 'MarkerFaceColor', 'm');
        
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

% Footplate torque (acting on the ankle) during 0–50% gait:
% heel friction torque can likely be ignored.
function tau_GRF = ground_reaction_torque(t)
    % t: input time (s), scalar or vector

    % --- Simulation settings ---
    feet_length = 0.25;  % Foot length (m)
    Body_weight = 60;    % Body weight (kg)
    T_stance = 0.5;      % Stance duration (s)

    % --- Original r_x data (% foot length) ---
    r_x = [10 15 22.5 27.5 30 32.5 37.5 42 45 51 56 60 65 70 72.5 74.5 76]; % 17 points
    x_old = linspace(0, T_stance, length(r_x));   % 0–0.5 s of stance

    % --- Interpolate to 10 time points (or more) ---
    t_interp = linspace(0, T_stance, 10);
    r_x_interp = interp1(x_old, r_x, t_interp, 'linear');
    r_x_net = r_x_interp - 25;                    % Ankle at 25%
    r_x_actual = r_x_net * feet_length / 100;     % meters

    % --- Vertical GRF (as BW ratio) ---
    F_y = [0 9 12 11.5 10.5 9.5 9 9.5 10.5 12 11]; % 11 points
    F_y_interp = interp1(linspace(0, T_stance, length(F_y)), F_y, t_interp);
    F_y_actual = F_y_interp * Body_weight;

    % --- Torque vector for first 0.5 s ---
    tau_half = r_x_actual .* F_y_actual;  % 2D r × F (scalar)

    % --- Output torque over time ---
    tau_GRF = zeros(size(t));   % Initialize

    % Interpolate for t in [0, 0.5]
    idx = t <= T_stance;
    tau_GRF(idx) = interp1(t_interp, tau_half, t(idx), 'linear', 0);  % 0 if out-of-range
end

% Toe torque during 50%–60% of gait
function tau_toe_GRF = toe_ground_reaction_torque(t)
    % t: input time (s), scalar or vector

    % --- Simulation settings ---
    feet_length = 0.25;  % Foot length (m)
    Body_weight = 60;    % Body weight (kg)
    T_start = 0.5;       % Toe support start time
    T_stance = 0.1;      % Toe support duration (s)

    % --- Time nodes and interpolation timeline (0.5–0.6 s) ---
    t_interp = linspace(T_start, T_start + T_stance, 10);

    % --- Original r_x data (% foot length) ---
    r_x = [79 83 91];  % Force application point moves toward the toe
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

    % Indices within toe support interval
    idx = (t >= T_start) & (t <= T_start + T_stance);

    % Interpolate
    tau_toe_GRF(idx) = interp1(t_interp, tau_toe_half, t(idx), 'linear', 0);
end

% Estimated leg-produced torque on the ankle over the gait cycle
function tau_leg_GRF = leg_ground_reaction_torque(t)
    % t: input time (s), scalar or vector

    % --- Simulation settings ---
    Body_weight = 60;  % Body weight (kg)
    T_stance = 1;      % Stance duration (s)

    % --- Interpolation timeline ---
    t_interp = linspace(0, T_stance, 20);

    % r_x as fraction of foot length (negative = behind ankle)
    r_x = [-5  -2.5   0    1    2      4   6   8   10   12  13 14  15 0 0 0 0 0 0 0 0]/100;
    F_y = [0     9    12  11.5 10.5   9.5  9   9.5 10.5 12  11  6  1  0 0 0 0 0 0 0 0]; % 21 points
    r_x_interp = interp1(linspace(0, T_stance, length(r_x)), r_x, t_interp);

    % Vertical GRF (BW ratio)
    F_y_interp = interp1(linspace(0, T_stance, length(F_y)), F_y, t_interp);
    F_y_actual = F_y_interp * Body_weight;

    tau_leg_half = r_x_interp .* F_y_actual;  % 2D r × F (scalar)

    % --- Output over time ---
    tau_leg_GRF = zeros(size(t));  % Initialize

    idx = t <= T_stance;
    tau_leg_GRF(idx) = interp1(t_interp, tau_leg_half, t(idx), 'linear', 0);
end

% Gait 50%–60% toe push-off: reaction torque of the toe on the footplate (seen by ankle)
function toe_ankle = toe_ankle_ground_reaction_torque(t)
    % t: input time (s), scalar or vector

    % --- Simulation settings ---      
    T_start = 0.5; 
    T_stance = 0.1;             

    % Time vector corresponding to M_toe_ankle points
    % M_toe_ankle has 3 points → t_interp has 3 points
    t_interp = linspace(0, T_stance, length([70 40 20]));
    M_toe_ankle = [70 40 20];

    toe_ankle = zeros(size(t));   % Initialize
    
    % Indices within support phase interval
    idx = (t >= T_start) & (t <= T_start + T_stance);
    
    % Shift to local time for interpolation
    t_local = t(idx) - T_start;
    
    % Interpolate (0 outside interval)
    toe_ankle(idx) = interp1(t_interp, M_toe_ankle, t_local, 'linear', 0);
end

function tau = pyStyleCross2D(a, b)
    % Input: a = [x1,y1], b = [x2,y2]
    % Output: same as np.cross([x1,y1,0], [x2,y2,0])[2]
    % Explicit 3D cross product projection
    cross_3d = cross([a(1), a(2), 0], [b(1), b(2), 0]);
    tau = cross_3d(3); % Take Z component
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

