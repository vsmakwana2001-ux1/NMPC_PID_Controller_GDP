%% plot_GDP_sweep_results.m
%  Professional plots for GDP sweep results.
%  Generates 5 figures:
%    1a. Peak Lateral Deviation – Stanley controllers
%    1b. Peak Lateral Deviation – Pure Pursuit, MPC, MPC Combined
%    2.  Peak Longitudinal Deviation (all controllers)
%    3.  MPC+LQR: Lateral & Longitudinal vs Speed
%    4.  MPC Combined: Lateral & Longitudinal vs Speed
%
%  Usage:  plot_GDP_sweep_results(T)

function plot_GDP_sweep_results(T)

    % ---- simple sequential palette (light-to-dark grey-blue) ----
    colors = [
        0.78  0.82  0.87;   % 0.5 m/s
        0.62  0.68  0.76;   % 2.0 m/s
        0.49  0.55  0.65;   % 3.0 m/s
        0.38  0.43  0.55;   % 4.0 m/s
        0.28  0.33  0.47;   % 5.0 m/s
        0.19  0.24  0.38;   % 7.0 m/s
        0.10  0.14  0.28;   % 10.0 m/s
    ];

    speeds = [0.5, 2.0, 3.0, 4.0, 5.0, 7.0, 10.0];
    speed_labels = { ...
        '0.5 m/s (const)', ...
        '2.0 m/s (const)', ...
        '3.0 m/s (profile)', ...
        '4.0 m/s (profile)', ...
        '5.0 m/s (profile)', ...
        '7.0 m/s (profile)', ...
        '10.0 m/s (const)'};

    controllers = {"stanley+pid","stanley+lqr", ...
                   "pp+pid","pp+lqr", ...
                   "mpc+pid","mpc+lqr","mpc_combined"};
    ctrl_display = {"Stanley + PID","Stanley + LQR", ...
                    "Pure Pursuit + PID","Pure Pursuit + LQR", ...
                    "MPC + PID","MPC + LQR","MPC Combined"};
    n_ctrl  = numel(controllers);
    n_speed = numel(speeds);

    % ---- build data matrices (controllers x speeds) ----
    lat_data = NaN(n_ctrl, n_speed);
    lon_data = NaN(n_ctrl, n_speed);

    for ci = 1:n_ctrl
        for si = 1:n_speed
            idx = T.Controller == string(controllers{ci}) & ...
                  abs(T.Peak_Speed_mps - speeds(si)) < 0.01;
            if any(idx)
                lat_data(ci,si) = T.Peak_CTE_m(idx);
                lon_data(ci,si) = T.Peak_Lon_Dev_m(idx);
            end
        end
    end

    % ---- common figure settings ----
    font_name  = 'Times New Roman';
    font_title = 13;
    font_axis  = 11;
    font_leg   = 8.5;
    bar_width  = 0.85;
    col_lat    = [0.20 0.30 0.50];
    col_lon    = [0.45 0.45 0.45];

    % ---- index sets for splitting lateral plots ----
    idx_stanley = [1 2];           % Stanley + PID, Stanley + LQR
    idx_others  = [3 4 5 6 7];    % PP+PID, PP+LQR, MPC+PID, MPC+LQR, MPC Combined

    % =====================================================================
    %  FIGURE 1a - Peak Lateral Deviation (Stanley controllers only)
    % =====================================================================
    fig1a = figure('Units','centimeters','Position',[2 14 22 13],...
        'Color','w','PaperPositionMode','auto');
    ax1a = axes(fig1a);
    b1a = bar(ax1a, lat_data(idx_stanley,:), bar_width, 'EdgeColor','none');
    for k = 1:n_speed
        b1a(k).FaceColor = 'flat';
        b1a(k).CData = repmat(colors(k,:), numel(idx_stanley), 1);
    end
    set(ax1a, 'XTickLabel', ctrl_display(idx_stanley), 'XTickLabelRotation', 15, ...
        'FontName', font_name, 'FontSize', font_axis, ...
        'Box','off', 'TickDir','out', 'LineWidth', 0.6);
    ylabel(ax1a, 'Peak Cross-Track Error (m)', 'FontSize', font_axis, 'FontName', font_name);
    title(ax1a, 'Peak Lateral Deviation — Stanley Controllers', ...
        'FontSize', font_title, 'FontName', font_name, 'FontWeight','bold');
    lg1a = legend(ax1a, speed_labels, 'Location','northeast', ...
        'FontSize', font_leg, 'FontName', font_name, 'NumColumns', 2);
    lg1a.Box = 'off';
    ax1a.YGrid = 'on'; ax1a.XGrid = 'off';
    ax1a.GridAlpha = 0.20; ax1a.GridLineStyle = '-';

    exportgraphics(fig1a, fullfile(pwd,'peak_lateral_deviation_stanley.png'), 'Resolution', 300);

    % =====================================================================
    %  FIGURE 1b - Peak Lateral Deviation (PP, MPC, MPC Combined)
    % =====================================================================
    fig1b = figure('Units','centimeters','Position',[26 14 28 13],...
        'Color','w','PaperPositionMode','auto');
    ax1b = axes(fig1b);
    b1b = bar(ax1b, lat_data(idx_others,:), bar_width, 'EdgeColor','none');
    for k = 1:n_speed
        b1b(k).FaceColor = 'flat';
        b1b(k).CData = repmat(colors(k,:), numel(idx_others), 1);
    end
    set(ax1b, 'XTickLabel', ctrl_display(idx_others), 'XTickLabelRotation', 25, ...
        'FontName', font_name, 'FontSize', font_axis, ...
        'Box','off', 'TickDir','out', 'LineWidth', 0.6);
    ylabel(ax1b, 'Peak Cross-Track Error (m)', 'FontSize', font_axis, 'FontName', font_name);
    title(ax1b, 'Peak Lateral Deviation — PP, MPC & MPC Combined', ...
        'FontSize', font_title, 'FontName', font_name, 'FontWeight','bold');
    lg1b = legend(ax1b, speed_labels, 'Location','northeast', ...
        'FontSize', font_leg, 'FontName', font_name, 'NumColumns', 2);
    lg1b.Box = 'off';
    ax1b.YGrid = 'on'; ax1b.XGrid = 'off';
    ax1b.GridAlpha = 0.20; ax1b.GridLineStyle = '-';

    exportgraphics(fig1b, fullfile(pwd,'peak_lateral_deviation_others.png'), 'Resolution', 300);

    % =====================================================================
    %  FIGURE 2 - Peak Longitudinal Deviation (all controllers)
    % =====================================================================
    fig2 = figure('Units','centimeters','Position',[2 0 30 13],...
        'Color','w','PaperPositionMode','auto');
    ax2 = axes(fig2);
    b2 = bar(ax2, lon_data, bar_width, 'EdgeColor','none');
    for k = 1:n_speed
        b2(k).FaceColor = 'flat';
        b2(k).CData = repmat(colors(k,:), n_ctrl, 1);
    end
    set(ax2, 'XTickLabel', ctrl_display, 'XTickLabelRotation', 25, ...
        'FontName', font_name, 'FontSize', font_axis, ...
        'Box','off', 'TickDir','out', 'LineWidth', 0.6);
    ylabel(ax2, 'Peak Longitudinal Deviation (m)', 'FontSize', font_axis, 'FontName', font_name);
    title(ax2, 'Peak Longitudinal Deviation by Controller Combination', ...
        'FontSize', font_title, 'FontName', font_name, 'FontWeight','bold');
    lg2 = legend(ax2, speed_labels, 'Location','northeast', ...
        'FontSize', font_leg, 'FontName', font_name, 'NumColumns', 2);
    lg2.Box = 'off';
    ax2.YGrid = 'on'; ax2.XGrid = 'off';
    ax2.GridAlpha = 0.20; ax2.GridLineStyle = '-';

    exportgraphics(fig2, fullfile(pwd,'peak_longitudinal_deviation.png'), 'Resolution', 300);

    % ---- x-axis tick labels for figures 3 & 4 ----
    xtick_labels = {'0.5\newline(const)','2.0\newline(const)', ...
                    '3.0\newline(profile)','4.0\newline(profile)', ...
                    '5.0\newline(profile)','7.0\newline(profile)', ...
                    '10.0\newline(const)'};

    % =====================================================================
    %  FIGURE 3 - MPC+LQR: Lateral & Longitudinal vs Speed
    % =====================================================================
    lat_mpc_lqr = NaN(1, n_speed);
    lon_mpc_lqr = NaN(1, n_speed);
    for si = 1:n_speed
        idx = T.Controller == "mpc+lqr" & abs(T.Peak_Speed_mps - speeds(si)) < 0.01;
        if any(idx)
            lat_mpc_lqr(si) = T.Peak_CTE_m(idx);
            lon_mpc_lqr(si) = T.Peak_Lon_Dev_m(idx);
        end
    end

    fig3 = figure('Units','centimeters','Position',[32 12 22 13],...
        'Color','w','PaperPositionMode','auto');
    ax3 = axes(fig3);
    hold(ax3, 'on');

    plot(ax3, speeds, lat_mpc_lqr, '-o', 'Color', col_lat, ...
        'MarkerFaceColor', col_lat, 'MarkerSize', 6, 'LineWidth', 1.5, ...
        'DisplayName', 'Peak Lateral Deviation');
    plot(ax3, speeds, lon_mpc_lqr, '-s', 'Color', col_lon, ...
        'MarkerFaceColor', col_lon, 'MarkerSize', 6, 'LineWidth', 1.5, ...
        'DisplayName', 'Peak Longitudinal Deviation');

    set(ax3, 'XTick', speeds, 'XTickLabel', xtick_labels, ...
        'FontName', font_name, 'FontSize', font_axis, ...
        'Box','off', 'TickDir','out', 'LineWidth', 0.6);
    xlabel(ax3, 'Reference Speed (m/s)', 'FontSize', font_axis, 'FontName', font_name);
    ylabel(ax3, 'Deviation (m)', 'FontSize', font_axis, 'FontName', font_name);
    title(ax3, 'MPC + LQR: Deviation vs Speed', ...
        'FontSize', font_title, 'FontName', font_name, 'FontWeight','bold');
    lg3 = legend(ax3, 'Location','northwest', 'FontSize', font_leg, 'FontName', font_name);
    lg3.Box = 'off';
    grid(ax3, 'on'); ax3.GridAlpha = 0.20;
    xlim(ax3, [0 11]);

    exportgraphics(fig3, fullfile(pwd,'mpc_lqr_deviation_vs_speed.png'), 'Resolution', 300);

    % =====================================================================
    %  FIGURE 4 - MPC Combined: Lateral & Longitudinal vs Speed
    % =====================================================================
    lat_mpc_comb = NaN(1, n_speed);
    lon_mpc_comb = NaN(1, n_speed);
    for si = 1:n_speed
        idx = T.Controller == "mpc_combined" & abs(T.Peak_Speed_mps - speeds(si)) < 0.01;
        if any(idx)
            lat_mpc_comb(si) = T.Peak_CTE_m(idx);
            lon_mpc_comb(si) = T.Peak_Lon_Dev_m(idx);
        end
    end

    fig4 = figure('Units','centimeters','Position',[32 0 22 13],...
        'Color','w','PaperPositionMode','auto');
    ax4 = axes(fig4);
    hold(ax4, 'on');

    plot(ax4, speeds, lat_mpc_comb, '-o', 'Color', col_lat, ...
        'MarkerFaceColor', col_lat, 'MarkerSize', 6, 'LineWidth', 1.5, ...
        'DisplayName', 'Peak Lateral Deviation');
    plot(ax4, speeds, lon_mpc_comb, '-s', 'Color', col_lon, ...
        'MarkerFaceColor', col_lon, 'MarkerSize', 6, 'LineWidth', 1.5, ...
        'DisplayName', 'Peak Longitudinal Deviation');

    set(ax4, 'XTick', speeds, 'XTickLabel', xtick_labels, ...
        'FontName', font_name, 'FontSize', font_axis, ...
        'Box','off', 'TickDir','out', 'LineWidth', 0.6);
    xlabel(ax4, 'Reference Speed (m/s)', 'FontSize', font_axis, 'FontName', font_name);
    ylabel(ax4, 'Deviation (m)', 'FontSize', font_axis, 'FontName', font_name);
    title(ax4, 'MPC Combined: Deviation vs Speed', ...
        'FontSize', font_title, 'FontName', font_name, 'FontWeight','bold');
    lg4 = legend(ax4, 'Location','northwest', 'FontSize', font_leg, 'FontName', font_name);
    lg4.Box = 'off';
    grid(ax4, 'on'); ax4.GridAlpha = 0.20;
    xlim(ax4, [0 11]);

    exportgraphics(fig4, fullfile(pwd,'mpc_combined_deviation_vs_speed.png'), 'Resolution', 300);

    fprintf('\nAll 5 figures saved as PNG (300 dpi).\n');
end