% =========================================================================
% PLOT_RESULTS.M
% Comprehensive visualization of 6-DOF missile simulation results
%
% PLOTS GENERATED:
%   Figure 1: 3D Trajectory (missile + target paths)
%   Figure 2: Velocities (u, v, w body components + total airspeed)
%   Figure 3: Pitch angle tracking (commanded vs. actual + error)
%   Figure 4: Control input (elevator deflection)
%   Figure 5: ADRC disturbance estimation (ESO output)
%   Figure 6: Miss distance over time
%   Figure 7: Aerodynamic angles (alpha, beta)
%   Figure 8: Angular rates (p, q, r)
%
% INPUTS:
%   t           - Time vector (s)
%   X_hist      - State history matrix [12 × N]
%   de_hist     - Elevator deflection history [1 × N] (rad)
%   theta_cmd   - Commanded pitch angle history [1 × N] (rad)
%   dist_est    - ESO disturbance estimate history [1 × N] (rad/s²)
%   target_hist - Target position history [3 × N] (NED, m)
%   miss_dist   - Miss distance history [1 × N] (m)
%   adrc        - ADRC parameter struct (for annotation)
% =========================================================================

function plot_results(t, X_hist, de_hist, theta_cmd, dist_est, target_hist, miss_dist, adrc)

    fprintf('Generating plots...\n');

    %% ---- Extract states ------------------------------------------------
    u     = X_hist(1,:);   v     = X_hist(2,:);   w     = X_hist(3,:);
    p     = X_hist(4,:);   q     = X_hist(5,:);   r     = X_hist(6,:);
    phi   = X_hist(7,:);   theta = X_hist(8,:);   psi   = X_hist(9,:);
    xI    = X_hist(10,:);  yI    = X_hist(11,:);  zI    = X_hist(12,:);

    % Derived quantities
    V_total = sqrt(u.^2 + v.^2 + w.^2);   % Total airspeed (m/s)
    alpha   = atan2(w, u);                  % Angle of attack (rad)
    beta    = asin(v ./ max(V_total, 1));   % Sideslip angle (rad)

    % Altitude (positive up, in meters)
    alt_m   = -zI;          % NED: altitude = -z_down
    alt_t   = -target_hist(3,:);

    % Gust window annotation
    gust_t_start = 5.0;
    gust_t_end   = 8.0;

    % Color palette
    c_blue   = [0.12, 0.47, 0.71];
    c_red    = [0.84, 0.15, 0.16];
    c_green  = [0.17, 0.63, 0.17];
    c_orange = [1.00, 0.50, 0.05];
    c_purple = [0.58, 0.40, 0.74];

    %% ==================================================================
    %  FIGURE 1: 3D TRAJECTORY
    % ==================================================================
    figure('Name','3D Trajectory','NumberTitle','off','Position',[50 500 750 550]);
    hold on; grid on; box on;

    % Missile trajectory (NED → plot as North/East/Altitude)
    plot3(xI, yI, alt_m, 'Color', c_blue, 'LineWidth', 2.5, 'DisplayName', 'Missile');

    % Target trajectory
    plot3(target_hist(1,:), target_hist(2,:), alt_t, ...
          'Color', c_red, 'LineWidth', 2.0, 'LineStyle', '--', 'DisplayName', 'Target');

    % Start/End markers
    plot3(xI(1), yI(1), alt_m(1), 'o', 'Color', c_blue, ...
          'MarkerSize', 10, 'MarkerFaceColor', c_blue, 'DisplayName', 'Launch');
    plot3(xI(end), yI(end), alt_m(end), 's', 'Color', c_blue, ...
          'MarkerSize', 10, 'MarkerFaceColor', c_blue, 'DisplayName', 'Impact Point');
    plot3(target_hist(1,1), target_hist(2,1), alt_t(1), '^', 'Color', c_red, ...
          'MarkerSize', 10, 'MarkerFaceColor', c_red, 'DisplayName', 'Target Start');
    plot3(target_hist(1,end), target_hist(2,end), alt_t(end), 'x', 'Color', c_red, ...
          'MarkerSize', 12, 'LineWidth', 2.5, 'DisplayName', 'Intercept');

    xlabel('North (m)', 'FontSize', 12);
    ylabel('East (m)',  'FontSize', 12);
    zlabel('Altitude (m)', 'FontSize', 12);
    title('6-DOF Missile Trajectory — Proportional Navigation Guidance', ...
          'FontSize', 13, 'FontWeight', 'bold');
    legend('Location', 'northwest', 'FontSize', 10);
    view(35, 25);
    set(gca, 'FontSize', 11);

    %% ==================================================================
    %  FIGURE 2: VELOCITIES
    % ==================================================================
    figure('Name','Velocities','NumberTitle','off','Position',[50 50 900 600]);

    subplot(2,2,1);
    plot(t, u, 'Color', c_blue, 'LineWidth', 1.8);
    ylabel('u (m/s)'); xlabel('Time (s)');
    title('Forward Body Velocity (u)'); grid on;
    add_gust_shade(gust_t_start, gust_t_end, ylim());

    subplot(2,2,2);
    plot(t, v, 'Color', c_orange, 'LineWidth', 1.8);
    ylabel('v (m/s)'); xlabel('Time (s)');
    title('Lateral Body Velocity (v)'); grid on;

    subplot(2,2,3);
    plot(t, w, 'Color', c_red, 'LineWidth', 1.8);
    ylabel('w (m/s)'); xlabel('Time (s)');
    title('Vertical Body Velocity (w)'); grid on;
    add_gust_shade(gust_t_start, gust_t_end, ylim());

    subplot(2,2,4);
    plot(t, V_total, 'Color', c_purple, 'LineWidth', 2.0);
    ylabel('|V| (m/s)'); xlabel('Time (s)');
    title('Total Airspeed'); grid on;
    sgtitle('Missile Velocity Components (Body Frame)', 'FontSize', 13, 'FontWeight','bold');

    %% ==================================================================
    %  FIGURE 3: PITCH ANGLE TRACKING (ADRC Performance)
    % ==================================================================
    figure('Name','Pitch Tracking','NumberTitle','off','Position',[820 500 850 600]);

    subplot(2,1,1);
    hold on; grid on;
    plot(t, rad2deg(theta_cmd), 'r--', 'LineWidth', 2.0, 'DisplayName', '\theta_{cmd} (Guidance)');
    plot(t, rad2deg(theta),     'b-',  'LineWidth', 1.8, 'DisplayName', '\theta_{actual}');
    ylabel('Pitch Angle (°)'); xlabel('Time (s)');
    title(sprintf('Pitch Tracking: ADRC (\\omega_c=%.0f rad/s, \\omega_o=%.0f rad/s)', ...
          adrc.omega_c, adrc.omega_o));
    legend('Location','best'); ylim([-45 45]);
    % Annotate gust window
    yl = ylim();
    patch([gust_t_start gust_t_end gust_t_end gust_t_start], ...
          [yl(1) yl(1) yl(2) yl(2)], [1 0.8 0.8], 'FaceAlpha', 0.3, ...
          'EdgeColor', 'none', 'DisplayName', 'Gust Window');

    subplot(2,1,2);
    theta_err = rad2deg(theta_cmd - theta);
    plot(t, theta_err, 'Color', c_orange, 'LineWidth', 1.5);
    hold on;
    yline(0, 'k--', 'LineWidth', 1.0);
    ylabel('Tracking Error (°)'); xlabel('Time (s)');
    title('Pitch Tracking Error'); grid on;
    sgtitle('ADRC Pitch Channel Performance', 'FontSize', 13, 'FontWeight','bold');

    %% ==================================================================
    %  FIGURE 4: CONTROL INPUT (Elevator Deflection)
    % ==================================================================
    figure('Name','Control Input','NumberTitle','off','Position',[820 50 850 400]);
    hold on; grid on;
    plot(t, rad2deg(de_hist), 'Color', c_green, 'LineWidth', 1.8);
    yline(rad2deg(adrc.de_max),  'r--', 'LineWidth', 1.5, 'Label', '+\delta_{e,max}');
    yline(-rad2deg(adrc.de_max), 'r--', 'LineWidth', 1.5, 'Label', '-\delta_{e,max}');
    yline(0, 'k:', 'LineWidth', 1.0);
    xlabel('Time (s)'); ylabel('Elevator \delta_e (°)');
    title('Control Input: Elevator Deflection Command', 'FontSize', 13, 'FontWeight','bold');
    ylim([-30 30]); set(gca,'FontSize',11);

    %% ==================================================================
    %  FIGURE 5: ESO DISTURBANCE ESTIMATION
    % ==================================================================
    figure('Name','Disturbance Estimation','NumberTitle','off','Position',[50 -400 900 500]);

    subplot(2,1,1);
    plot(t, dist_est, 'Color', c_purple, 'LineWidth', 1.8);
    hold on; grid on;
    % Mark gust window
    yl = ylim(); yl = [-max(abs(yl))*1.2, max(abs(yl))*1.2];
    patch([gust_t_start gust_t_end gust_t_end gust_t_start], ...
          [yl(1) yl(1) yl(2) yl(2)], [0.8 0.8 1.0], 'FaceAlpha', 0.3, ...
          'EdgeColor','none');
    xlabel('Time (s)'); ylabel('z_3 (rad/s²)');
    title('ESO Disturbance Estimate z_3(t) — Active Disturbance Rejection');
    legend({'ESO Estimate z_3', 'Gust Window'}, 'Location','best'); grid on;

    subplot(2,1,2);
    plot(t, rad2deg(q), 'Color', c_blue, 'LineWidth', 1.5);
    hold on;
    xlabel('Time (s)'); ylabel('Pitch Rate q (°/s)');
    title('Pitch Rate (Controlled Response)'); grid on;
    sgtitle('ADRC Extended State Observer Output', 'FontSize', 13, 'FontWeight','bold');

    %% ==================================================================
    %  FIGURE 6: MISS DISTANCE
    % ==================================================================
    figure('Name','Miss Distance','NumberTitle','off','Position',[820 -400 850 400]);
    semilogy(t, miss_dist, 'Color', c_red, 'LineWidth', 2.0);
    hold on; grid on;
    yline(20, 'k--', 'LineWidth', 1.5, 'Label', 'Kill radius (20 m)');
    xlabel('Time (s)'); ylabel('Range to Target (m)');
    title('Missile-Target Range (Miss Distance)', 'FontSize', 13, 'FontWeight','bold');
    set(gca, 'FontSize', 11);

    %% ==================================================================
    %  FIGURE 7: AERODYNAMIC ANGLES
    % ==================================================================
    figure('Name','Aero Angles','NumberTitle','off','Position',[50 -800 900 450]);

    subplot(1,2,1);
    plot(t, rad2deg(alpha), 'Color', c_blue, 'LineWidth', 1.8);
    xlabel('Time (s)'); ylabel('\alpha (°)');
    title('Angle of Attack'); grid on;
    yline(0,'k--');

    subplot(1,2,2);
    plot(t, rad2deg(beta), 'Color', c_orange, 'LineWidth', 1.8);
    xlabel('Time (s)'); ylabel('\beta (°)');
    title('Sideslip Angle'); grid on;
    yline(0,'k--');
    sgtitle('Aerodynamic Angles', 'FontSize', 13, 'FontWeight','bold');

    %% ==================================================================
    %  FIGURE 8: ANGULAR RATES
    % ==================================================================
    figure('Name','Angular Rates','NumberTitle','off','Position',[820 -800 850 450]);

    subplot(1,3,1);
    plot(t, rad2deg(p), 'Color', c_blue, 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('p (°/s)'); title('Roll Rate'); grid on;

    subplot(1,3,2);
    plot(t, rad2deg(q), 'Color', c_red, 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('q (°/s)'); title('Pitch Rate'); grid on;

    subplot(1,3,3);
    plot(t, rad2deg(r), 'Color', c_green, 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('r (°/s)'); title('Yaw Rate'); grid on;
    sgtitle('Body Angular Rates (p, q, r)', 'FontSize', 13, 'FontWeight','bold');

    %% ---- Print summary statistics -------------------------------------
    fprintf('\n=== SIMULATION SUMMARY ===\n');
    fprintf('Final miss distance:      %.2f m\n', miss_dist(end));
    fprintf('Max elevator deflection:  %.2f °\n', max(abs(rad2deg(de_hist))));
    fprintf('Max pitch angle:          %.2f °\n', max(abs(rad2deg(theta))));
    fprintf('Max angle of attack:      %.2f °\n', max(abs(rad2deg(alpha))));
    fprintf('Max pitch tracking error: %.2f °\n', max(abs(rad2deg(theta_cmd - theta))));
    fprintf('Max disturbance estimate: %.2f rad/s²\n', max(abs(dist_est)));
    fprintf('Peak airspeed:            %.1f m/s (Mach %.2f)\n', ...
            max(V_total), max(V_total)/340);
    fprintf('==========================\n\n');

    fprintf('All figures generated successfully.\n');
end

%% ---- Helper: shade a time window on current axes ----------------------
function add_gust_shade(t1, t2, yl)
    if isempty(yl), yl = [-1 1]; end
    patch([t1 t2 t2 t1], [yl(1) yl(1) yl(2) yl(2)], ...
          [1.0 0.8 0.8], 'FaceAlpha', 0.3, 'EdgeColor', 'none', ...
          'HandleVisibility', 'off');
end
