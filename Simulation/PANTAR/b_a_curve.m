% omega = [0 1 2 3 4 5];
% ba = [5 4 2.75 2.07 1.55 1.24];
%
% % Quadratic polynomial fit
% p = polyfit(omega, ba, 2);
%
% % Dense omega for smooth fitted curve
% omega_fit = linspace(min(omega), max(omega), 100);
%
% % Evaluate fitted damping coefficients
% ba_fit = polyval(p, omega_fit);
%
% % Plot
% plot(omega, ba, 'ro', 'MarkerSize', 8, 'DisplayName', 'Original data'); hold on;
% plot(omega_fit, ba_fit, 'b-', 'LineWidth', 2, 'DisplayName', 'Quadratic fit');
% xlabel('Angular velocity \omega (rad/s)');
% ylabel('Damping coefficient b_a');
% title('Damping coefficient vs angular velocity');
% legend show
% grid on

% ---- Damping-coefficient function ----
function ba = damping_coefficient(omega)
    % Exponential fit for the damping coefficient.
    % Input:
    %   omega - angular velocity (rad/s), scalar or vector
    % Output:
    %   ba    - damping coefficient corresponding to omega

    % Fit parameters
    a = 3;        % lower-bounded at 3 (was 5.04)
    b = -0.31;

    % Compute damping coefficient
    ba = a * exp(b * omega);

    % Zero damping for very small speeds
    ba(omega < 0.3) = 0;
end

% ---- Plotting script ----
omega_vals = linspace(0, 6, 300);   % omega from 0 to 6 rad/s
ba_vals = damping_coefficient(omega_vals);

figure;
plot(omega_vals, ba_vals, 'b-', 'LineWidth', 2);
xlabel('\omega (rad/s)');
ylabel('Damping coefficient b_a');
title('Damping coefficient b_a vs angular velocity \omega');
grid on;
