% Main simulation function
[t, y] = prosthetic_foot_dynamics_sim();

% Visualization animation
visualize_prosthetic_motion(t, y);

% ================= Main dynamics simulation function =================
function [t, y] = prosthetic_foot_dynamics_sim()
    % Parameter settings
    I_a = 0.05; % Ankle joint moment of inertia (kg·m^2)
    I_t = 0.01; % Toe joint moment of inertia (kg·m^2)
    k_a = 100;  % Ankle joint spring stiffness (Nm/rad)
    k_t = 50;   % Toe joint spring stiffness (Nm/rad)

    % Initial state: angles and angular velocities [theta_a; omega_a; theta_t; omega_t]
    y0 = [0; 0; 0; 0]; % Initial angle 0, velocity 0

    % Simulation time
    tspan = [0 5]; % 5 seconds

    % ODE solver
    [t, y] = ode45(@(t,y) dynamics(t,y,I_a,I_t,k_a,k_t), tspan, y0);

    % Plot joint angle curves
    figure;
    plot(t, y(:,1)*180/pi, 'b', 'LineWidth',2); hold on;
    plot(t, y(:,3)*180/pi, 'r', 'LineWidth',2);
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    legend('Ankle joint angle \theta_a','Toe joint angle \theta_t');
    title('Prosthetic foot joint angles over time');
    grid on;
end

% ================= State derivative function =================
function dydt = dynamics(t,y,I_a,I_t,k_a,k_t)
    % State variables
    theta_a = y(1); omega_a = y(2);
    theta_t = y(3); omega_t = y(4);

    % Motor torque inputs (example: ankle driven by sine wave, toe with simple control)
    tau_motor_a = 5*sin(2*pi*0.5*t); % Nm
    tau_motor_t = 2*(1 - theta_t);   % Toe joint proportional control to zero

    % Spring torques
    tau_spring_a = -k_a * theta_a;
    tau_spring_t = -k_t * theta_t;

    % Dynamics equations
    domega_a = (tau_motor_a + tau_spring_a) / I_a;
    domega_t = (tau_motor_t + tau_spring_t) / I_t;

    % Return derivative vector
    dydt = zeros(4,1);
    dydt(1) = omega_a;
    dydt(2) = domega_a;
    dydt(3) = omega_t;
    dydt(4) = domega_t;
end

% ================= Motion visualization function =================
function visualize_prosthetic_motion(t, y)
    % Prosthesis length parameters
    L1 = 0.2;  % Foot plate length (m)
    L2 = 0.05; % Toe length (m)

    figure;
    for i = 1:10:length(t)
        theta_a = y(i,1); % Ankle joint angle
        theta_t = y(i,3); % Toe joint angle

        % Position calculations
        p0 = [0; 0]; % Ankle joint as origin
        p1 = p0 + L1 * [cos(theta_a); sin(theta_a)];          % End of foot plate
        theta_toe = theta_a + theta_t;                        % Absolute angle
        p2 = p1 + L2 * [cos(theta_toe); sin(theta_toe)];      % End of toe

        % Plotting
        clf;
        hold on;
        plot([p0(1), p1(1)], [p0(2), p1(2)], 'b', 'LineWidth', 4); % Foot plate
        plot([p1(1), p2(1)], [p1(2), p2(2)], 'r', 'LineWidth', 4); % Toe
        plot(p0(1), p0(2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k'); % Ankle joint
        plot(p1(1), p1(2), 'ko', 'MarkerSize', 8); % Toe joint

        axis equal;
        axis([-0.1 0.3 -0.1 0.2]);
        title(sprintf('Time %.2f s', t(i)));
        xlabel('X (m)');
        ylabel('Y (m)');
        grid on;
        pause(0.02);
    end
end

