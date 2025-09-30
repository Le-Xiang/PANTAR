function tau_GRF = ground_reaction_torque(t)
    % t: input time (unit: seconds), can be scalar or vector

    % --- Simulation parameter settings ---
    feet_length = 0.3;           % Foot length (m)
    Body_weight = 1;             % Body weight (kg)
    T_stance = 0.6;               % Stance phase duration (s)

    % --- Original r_x data (assumed unit: % foot length) ---
    r_x = [10 15 22.5 27.5 30 32.5 37.5 42 45 51 56 60 65 70 72.5 74.5 76 78 81.5]; % 19 points, data from Center of Pressure Trajectory during Gait Figure 1, first row, first plot
    x_old = linspace(0, T_stance, length(r_x));   % Corresponds to stance phase 0~0.5s

    % --- Interpolate to 10 time points (or more) ---
    t_interp = linspace(0, T_stance, 10);
    r_x_interp = interp1(x_old, r_x, t_interp, 'linear');
    r_x_net = r_x_interp - 25;                    % Centered at the ankle (25%)
    r_x_actual = r_x_net * feet_length / 100;     % Convert to meters

    % --- Simulate vertical GRF, unit: BW (body weight) ratio ---
    F_y = [0 9 12 11.5 10.5 9.5 9 9.5 10.5 12 11 5 1]; % 13 points, data from Prediction of ground reaction forces during gait based on kinematics and a neural network model Figure 5, third row, first plot
    F_y_interp = interp1(linspace(0, T_stance, length(F_y)), F_y, t_interp);
    F_y_actual = F_y_interp * Body_weight;

    % --- Calculate the torque vector for the first 0.5 seconds ---
    tau_half = r_x_actual .* F_y_actual;  % Cross product: r × F (scalar)

    % --- Output tau at corresponding time points ---
    tau_GRF = zeros(size(t));             % Initialize to 0

    % Interpolate for points in t within [0, 0.5]
    idx = t <= T_stance;
    tau_GRF(idx) = interp1(t_interp, tau_half, t(idx), 'linear', 0);  % Interpolation, out-of-range values are 0
end
% The torque on the toe during 50%-60% of the gait cycle
function tau_toe_GRF = toe_ground_reaction_torque(t)
    % t: input time (unit: seconds), can be scalar or vector

    % --- Simulation parameter settings ---
    feet_length = 0.3;           % Foot length (m)
    Body_weight = 1;             % Body weight (kg)
    T_start = 0.5;                % Toe support phase start time
    T_stance = 0.1;               % Toe support phase duration (s)

    % --- Time nodes and interpolation time ---
    t_interp = linspace(T_start, T_start + T_stance, 10);  % Toe support period: 0.5~0.6s

    % --- Original r_x data (% foot length) ---
    r_x = [77 82 91];  % The force application point moves in the toe direction, data from Center of Pressure Trajectory during Gait Figure 1, first row, first plot
    t_r = linspace(T_start, T_start + T_stance, length(r_x));
    r_x_interp = interp1(t_r, r_x, t_interp, 'linear');

    % --- Relative position to the toe joint (toe at 75%) ---
    r_x_net = r_x_interp - 75;
    r_x_actual = r_x_net * feet_length / 100;  % Convert to meters

    % --- Vertical GRF (unit: body weight ratio) ---
    F_y = [4.5 3.5 0.5]; % Data from Kinetic Role of the Metatarsophalangeal Joint in Normal Walking: Joint Moment and Power Figure 5 (a) Vertical GRF
    t_f = linspace(T_start, T_start + T_stance, length(F_y));
    F_y_interp = interp1(t_f, F_y, t_interp, 'linear');
    F_y_actual = F_y_interp * Body_weight;

    % --- Calculate torque: tau = r × F (scalar multiplication in 2D) ---
    tau_toe_half = r_x_actual .* F_y_actual;

    % --- Output final result ---
    tau_toe_GRF = zeros(size(t));  % Initialize

    % Find the indices in t that are within the toe support period
    idx = (t >= T_start) & (t <= T_start + T_stance);

    % Interpolate output
    tau_toe_GRF(idx) = interp1(t_interp, tau_toe_half, t(idx), 'linear', 0);
end

% Simulated estimated torque of the leg on the ankle joint during the gait cycle
function tau_leg_GRF = leg_ground_reaction_torque(t)
    % t: input time (unit: seconds), can be scalar or vector

    % --- Simulation parameter settings --        % Foot length (m)
    Body_weight = 1;             % Body weight (kg)
    T_stance = 1;               % Stance phase duration (s)

    % --- Original r_x data (assumed unit: % foot length) ---
    t_interp = linspace(0, T_stance, 20);

    r_x = [-5  -2.5   0    1    2      4   6   8   10   12  13 14  15 0 0 0 0 0 0 0 0]/100;
    F_y = [0     9    12  11.5 10.5   9.5  9   9.5 10.5 12  11  6  1  0 0 0 0 0 0 0 0]; % 21 points
    r_x_interp = interp1(linspace(0, T_stance, length(r_x)), r_x, t_interp);


    % --- Simulate vertical GRF, unit: BW (body weight) ratio ---
    F_y_interp = interp1(linspace(0, T_stance, length(F_y)), F_y, t_interp);
    F_y_actual = F_y_interp * Body_weight;

    tau_leg_half = r_x_interp .* F_y_actual;  % Cross product: r × F (scalar)

    % --- Output tau at corresponding time points ---
    tau_leg_GRF = zeros(size(t));             % Initialize to 0

    idx = t <= T_stance;
    tau_leg_GRF(idx) = interp1(t_interp, tau_leg_half, t(idx), 'linear', 0);  % Interpolation, out-of-range values are 0
end


% The process of toe push-off during 50%-60% of the gait cycle, the reaction torque of the toe on the foot
function toe_ankle = toe_ankle_ground_reaction_torque(t)
    % t: input time (unit: seconds), can be scalar or vector

    % --- Simulation parameter settings --      
    T_start = 0.1; 
    T_stance = 0.5;  
    Body_weight = 1; 

    % Time vector corresponding to the points of M_toe_ankle

    M_toe_ankle = Body_weight*[0 0.005 0.01 0.015 0.025 0.04 0.07 0.1 0.125 0.11 0.025]; % Data from Kinetic Role of the Metatarsophalangeal Joint in Normal Walking: Joint Moment and Power Figure 5 (c) MP Joint Moment / Actually, the reaction torque of the toe joint obtained here can also be used
    
    % M_toe_ankle has 3 points, so t_interp should also have 3 corresponding points
    t_interp = linspace(0, T_stance, length(M_toe_ankle));

    toe_ankle = zeros(size(t));   % Initialize to 0
    
    % Find the indices in t that are within the support phase interval
    idx = (t >= T_start) & (t <= T_start + T_stance);
    
    % For these time points, subtract T_start to make them correspond to the t_interp interval
    t_local = t(idx) - T_start;
    
    % Interpolation calculation
    toe_ankle(idx) = interp1(t_interp, M_toe_ankle, t_local, 'linear', 0);  % Interpolation, out-of-range values are 0
end

% 定义时间向量 t（比如 0 到 1 秒，共 1000 个点）
t = linspace(0, 1, 1000);  

% 获取三个 torque 的数据
toe_ankle = toe_ankle_ground_reaction_torque(t);
tau_GRF = ground_reaction_torque(t);
tau_leg = leg_ground_reaction_torque(t);
tau_toe_GRF = toe_ground_reaction_torque(t);

tau = tau_GRF + toe_ankle;

% 画图
figure;
plot(t, toe_ankle, 'r', 'LineWidth', 1.5); hold on;
plot(t, tau_GRF, 'g', 'LineWidth', 1.5);
% plot(t, tau_leg, 'y', 'LineWidth', 1.5);
plot(t, tau, 'b', 'LineWidth', 1.5);
plot(t, tau_toe_GRF, 'k', 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Torque (Nm)');
title('Ground Reaction Torques Over Time');
legend('Toe-Ankle Reaction Torque', 'Ankle GRF Torque','Total Ankle Torque','Toe GRF Torque');
grid on;


% figure;
% plot(t, toe_ankle, 'r', 'LineWidth', 1.5); hold on;
% plot(t*100, tau_GRF, 'k', 'LineWidth', 1.5);
% plot(t, tau_leg, 'y', 'LineWidth', 1.5);
% plot(t, tau, 'b', 'LineWidth', 1.5);
% plot(t, tau_toe_GRF, 'k', 'LineWidth', 1.5);

xlabel('Gait Cycle %');
ylabel('Moment (Nm/kg)');
title('Ground Reaction Moment Over Gait Cycle');
legend('GRF Moment on Ankle Joint','GRF Moment on Toe Joint');
grid on;
