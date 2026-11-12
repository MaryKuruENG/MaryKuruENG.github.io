%% compare_pid_vs_assy.m  --  Simulink (pendulum_pid) vs Simscape (PendulumAssy), same PID gains
% Runs ONLY your two existing models and plots them on one figure. It never builds a model.
%   theta : blue     x : green     F : red      (solid = Simscape, dashed magenta = Simulink)
% Needs in this folder: params.m, pid_gains.mat, pendulum_pid.slx, PendulumAssy.slx,
%                       PendulumAssy_DataFile.m and the *_sldprt.STEP files.
% Output: fig_simulink_vs_simscape.png, simulink_vs_simscape.csv, printed table.

clearvars; clc; close all;
simulinkModel = 'pendulum_pid';     % Phase 2 Simulink model (linear plant + PID)
simscapeModel = 'PendulumAssy';     % Phase 3 Simscape model (imported CAD + PID)
Tend = 8;                           % simulation time [s]

for f = {[simulinkModel '.slx'], [simscapeModel '.slx'], 'params.m', 'pid_gains.mat', 'PendulumAssy_DataFile.m'}
    if ~isfile(f{1}), error('%s is not in the current folder (%s).', f{1}, pwd); end
end

% workspace data used by the two models
params;                              % M, m, l, I, g, th0, A, B, ...
load('pid_gains.mat');               % Kp, Ki, Kd, Tf  (used by pendulum_pid)
PendulumAssy_DataFile;               % smiData         (used by PendulumAssy)
Cfull = eye(4);  Dfull = zeros(4,1);  x0 = [0; 0; th0; 0];   % used by pendulum_pid

%% 1. Simulink model: log theta (Demux 3), x (Demux 1), F (PID output) -------------------
load_system(simulinkModel);
log_port([simulinkModel '/Demux'], 3, 'theta');
log_port([simulinkModel '/Demux'], 1, 'x');
log_port([simulinkModel '/PID'],   1, 'F');
o1 = sim(simulinkModel, 'StopTime', num2str(Tend), 'ReturnWorkspaceOutputs', 'on', ...
         'SignalLogging', 'on', 'SignalLoggingName', 'logsout');
[t1,  th1] = getsig(o1.logsout.get('theta').Values);
[tx1, x1 ] = getsig(o1.logsout.get('x').Values);
[tF1, F1 ] = getsig(o1.logsout.get('F').Values);

%% 2. Simscape model: log theta (Bias output), x (converter in m), F (PID Controller) --------
load_system(simscapeModel);
log_port([simscapeModel '/Bias'],           1, 'theta');
log_port(converter_with_unit(simscapeModel, 'm'), 1, 'x');
log_port([simscapeModel '/PID Controller'], 1, 'F');
o2 = sim(simscapeModel, 'StopTime', num2str(Tend), 'ReturnWorkspaceOutputs', 'on', ...
         'SignalLogging', 'on', 'SignalLoggingName', 'logsout');
[t2,  th2] = getsig(o2.logsout.get('theta').Values);
[tx2, x2 ] = getsig(o2.logsout.get('x').Values);
[tF2, F2 ] = getsig(o2.logsout.get('F').Values);
fprintf('theta(0): Simulink %.7f rad | Simscape %.7f rad\n', th1(1), th2(1));

%% 3. Common 1 ms time grid ----------------------------------------------------------------
tg  = (0:1e-3:min([t1(end) t2(end)])).';
th1g = interp1(t1, th1, tg);   th2g = interp1(t2, th2, tg);
x1g  = interp1(tx1, x1, tg);   x2g  = interp1(tx2, x2, tg);
F1g  = interp1(tF1, F1, tg);   F2g  = interp1(tF2, F2, tg);

%% 4. Figure (edit the titles / labels below as you like) -------------------------------------
blue = [0 0.45 0.74];  green = [0.13 0.55 0.13];  red = [0.85 0.10 0.10];
simCol = [0.85 0.00 0.85];           % Simulink line colour (magenta, visible on dark and white)
zeroCol = [0.5 0.5 0.5];             % zero reference lines (grey)
fig = figure('Name','Simulink vs Simscape','Units','centimeters','Position',[2 2 16 19]);
tlo = tiledlayout(4,1,'TileSpacing','compact','Padding','compact');
title(tlo, 'Comparison of Simscape and Simulink results', 'FontWeight','bold');

ax(1) = nexttile;
plot(tg, rad2deg(th2g), 'Color', blue, 'LineWidth', 2); hold on;
plot(tg, rad2deg(th1g), '--', 'Color', simCol, 'LineWidth', 1.6);  yline(0, ':', 'Color', zeroCol);
ylabel('\theta [deg]');  title('(a) Pendulum angle \theta');  grid on;
legend('Simscape', 'Simulink', 'Location', 'northeast');

ax(2) = nexttile;
plot(tg, x2g*1e3, 'Color', green, 'LineWidth', 2); hold on;
plot(tg, x1g*1e3, '--', 'Color', simCol, 'LineWidth', 1.6);
ylabel('x [mm]');  title('(b) Cart position x');  grid on;
legend('Simscape', 'Simulink', 'Location', 'southeast');

ax(3) = nexttile;
plot(tg, F2g, 'Color', red, 'LineWidth', 2); hold on;
plot(tg, F1g, '--', 'Color', simCol, 'LineWidth', 1.6);  yline(0, ':', 'Color', zeroCol);
ylabel('F [N]');  title('(c) Control force F');  grid on;
legend('Simscape', 'Simulink', 'Location', 'northeast');

ax(4) = nexttile;
plot(tg, rad2deg(th2g - th1g), 'Color', blue, 'LineWidth', 1.2);  yline(0, ':', 'Color', zeroCol);
ylabel('\Delta\theta [deg]');  xlabel('Time [s]');  grid on;
title('(d) Difference \theta_{Simscape} - \theta_{Simulink}');

linkaxes(ax, 'x');  xlim([0 4]);          % use xlim([0 Tend]) to show the full 8 s
useWhiteBackground = true;            % true -> white figure for the printed report
if useWhiteBackground
    try, theme(fig, 'light'); catch, set(fig, 'Color', 'w'); end
end
exportgraphics(fig, 'fig_simulink_vs_simscape.png', 'Resolution', 300, 'BackgroundColor', 'white');

%% 5. Numbers for the report ------------------------------------------------------------------
m1 = metrics(tg, th1g, x1g, F1g, th0);   m2 = metrics(tg, th2g, x2g, F2g, th0);
names = {'Initial angle [deg]'; 'Settling time 2% [s]'; 'Undershoot [% of th0]'; ...
         'Peak force [N]'; 'Cart peak |x| [mm]'; 'Cart final x [mm]'};
T = table(names, m1, m2, m2 - m1, 100*(m2 - m1)./max(abs(m1), eps), ...
    'VariableNames', {'Metric','Simulink','Simscape','Difference','Diff_pct'});
disp(T);  writetable(T, 'simulink_vs_simscape.csv');
fprintf('Max |dtheta| = %.4f deg | RMS = %.4f deg\n', ...
        max(abs(rad2deg(th2g - th1g))), sqrt(mean(rad2deg(th2g - th1g).^2)));

%% ======================================= local functions ==================================
function log_port(blk, port, name)       % name a block output signal and log it
    ph = get_param(blk, 'PortHandles');
    set_param(ph.Outport(port), 'Name', name, 'DataLogging', 'on');
end

function [t, y] = getsig(ts)              % timeseries -> column vectors, repeated times removed
    t = ts.Time(:);  y = squeeze(ts.Data);  y = y(:);
    [t, iu] = unique(t, 'last');  y = y(iu);
end

function blk = converter_with_unit(mdl, unit)   % PS-Simulink converter whose unit is `unit`
    b = find_system(mdl, 'SearchDepth', 1, 'Type', 'Block');
    for k = 1:numel(b)
        try
            if contains(get_param(b{k}, 'ReferenceBlock'), 'PS-Simulink') && ...
               strcmp(strtrim(get_param(b{k}, 'Unit')), unit)
                blk = b{k};  return
            end
        catch
        end
    end
    error('No PS-Simulink converter with unit "%s" found in %s.', unit, mdl);
end

function m = metrics(t, th, x, F, th0)
    k = find(abs(th) > 0.02*abs(th0), 1, 'last');
    ts = 0;  if ~isempty(k), ts = t(min(k+1, numel(t))); end
    m = [rad2deg(th(1)); ts; -100*min(th)/th0; max(abs(F)); 1e3*max(abs(x)); 1e3*x(end)];
end