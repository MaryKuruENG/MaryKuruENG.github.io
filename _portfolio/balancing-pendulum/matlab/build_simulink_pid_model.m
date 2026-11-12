%% build_simulink_pid_model.m -- builds, lays out and runs the Simulink model "pendulum_pid"
% Linearised state-space plant + PID Controller block, with a tidy block diagram,
% labelled scopes arranged side by side, and one results figure for the report.
% Run main_pid_inverted_pendulum.m first (it creates pid_gains.mat).
% Outputs: pendulum_pid.slx
%          fig2a_simulink_model.pdf / .png   (block diagram, sharp, for the report)
%          fig_simulink_results.png / .pdf   (theta, x, F on one white figure)

clear; close all; clc;
params;  load('pid_gains.mat');                  % A, B, th0 ... and Kp, Ki, Kd, Tf
Cfull = eye(4);  Dfull = zeros(4,1);             % plant outputs all 4 states [x xd th thd]
x0 = [0; 0; th0; 0];                             % start from the CAD tilt (1.37 deg)
Tend = 8;

mdl = 'pendulum_pid';
if bdIsLoaded(mdl), close_system(mdl, 0); end
if isfile([mdl '.slx']), delete([mdl '.slx']); end
new_system(mdl);  open_system(mdl);

%% 1. Blocks on a clean grid (signal flows left -> right, main line at y = 110) --------------
%           [left top right bottom]
add_block('simulink/Sources/Constant',   [mdl '/ThetaRef'], 'Value', '0', ...
          'Position', [40 95 70 125]);
add_block('simulink/Math Operations/Sum', [mdl '/Sum'], 'Inputs', '|+-', 'IconShape', 'round', ...
          'Position', [120 95 150 125]);
add_block('simulink/Continuous/PID Controller', [mdl '/PID'], ...
          'P', 'Kp', 'I', 'Ki', 'D', 'Kd', 'Controller', 'PID', 'TimeDomain', 'Continuous-time', ...
          'UseFilter', 'on', 'N', '1/Tf', 'InitialConditionForFilter', 'Kd*(-th0)', ... % no kick at t=0
          'Position', [200 85 290 135]);
add_block('simulink/Continuous/State-Space', [mdl '/Plant'], ...
          'A', 'A', 'B', 'B', 'C', 'Cfull', 'D', 'Dfull', 'X0', 'x0', ...
          'Position', [370 85 480 135]);
add_block('simulink/Signal Routing/Demux', [mdl '/Demux'], 'Outputs', '4', ...
          'DisplayOption', 'bar', 'Position', [540 70 545 150]);      % outputs at y = 80/100/120/140

% scopes on the right, one per signal, in the same order as the Demux outputs
add_block('simulink/Sinks/Scope', [mdl '/Force'],          'Position', [700  15 730  45]);
add_block('simulink/Sinks/Scope', [mdl '/Cart Position'],  'Position', [700  65 730  95]);
add_block('simulink/Sinks/Scope', [mdl '/Pendulum Angle'], 'Position', [700 145 730 175]);
% unused Demux outputs (cart and pendulum speeds) end in terminators -> no warnings
add_block('simulink/Sinks/Terminator', [mdl '/xdot'],     'Position', [600  92 615 108]);
add_block('simulink/Sinks/Terminator', [mdl '/thetadot'], 'Position', [600 132 615 148]);
set_param([mdl '/xdot'], 'ShowName', 'off');  set_param([mdl '/thetadot'], 'ShowName', 'off');

%% 2. Wires (smart autorouting gives straight lines and clean branch points) ---------------
r = {'autorouting', 'smart'};
add_line(mdl, 'ThetaRef/1', 'Sum/1',            r{:});
add_line(mdl, 'Sum/1',      'PID/1',            r{:});
add_line(mdl, 'PID/1',      'Plant/1',          r{:});
add_line(mdl, 'PID/1',      'Force/1',          r{:});   % branch: control force F
add_line(mdl, 'Plant/1',    'Demux/1',          r{:});
add_line(mdl, 'Demux/1',    'Cart Position/1',  r{:});
add_line(mdl, 'Demux/2',    'xdot/1',           r{:});
add_line(mdl, 'Demux/3',    'Pendulum Angle/1', r{:});
add_line(mdl, 'Demux/4',    'thetadot/1',       r{:});
add_line(mdl, 'Demux/3',    'Sum/2',            r{:});   % feedback of theta (runs under the blocks)

% signal names shown on the wires
name_out([mdl '/PID'],   1, 'F');
name_out([mdl '/Sum'],   1, 'e');
name_out([mdl '/Demux'], 1, 'x');
name_out([mdl '/Demux'], 3, 'theta');

% short annotation under the diagram
note = Simulink.Annotation([mdl '/Linearised cart-pendulum, PID on theta, theta(0) = 1.37 deg (CAD tilt)']);
note.Position = [200 200 560 220];  note.FontSize = 10;

%% 3. Scope settings: titles, units, grid, fixed axes, tiled on screen ---------------------
scr = get(0, 'ScreenSize');  w = round(scr(3)/3) - 10;  h = 330;  y = scr(4) - h - 90;
scope_setup([mdl '/Pendulum Angle'], 'Pendulum angle \theta', '\theta [rad]', [-0.01 0.025], [ 10      y w h]);
scope_setup([mdl '/Cart Position'],  'Cart position x',       'x [m]',        [-0.005 0.035], [ 20+w    y w h]);
scope_setup([mdl '/Force'],          'Control force F',       'F [N]',        [-20 110],      [ 30+2*w  y w h]);

%% 4. Solver, save, run ------------------------------------------------------------------
set_param(mdl, 'StopTime', num2str(Tend), 'Solver', 'ode45', 'MaxStep', '1e-3', ...
          'RelTol', '1e-6', 'SignalLogging', 'on', 'SignalLoggingName', 'logsout', ...
          'ZoomFactor', 'FitSystem');
save_system(mdl);
for s = {'Pendulum Angle', 'Cart Position', 'Force'}          % open the scopes BEFORE running,
    open_system([mdl '/' s{1}]);                               % so they draw while it simulates
end
drawnow;
set_param(mdl, 'SimulationCommand', 'start');                  % same as pressing Run
while ~strcmp(get_param(mdl, 'SimulationStatus'), 'stopped')   % wait until the run has finished
    pause(0.2);
end
out = sim(mdl, 'ReturnWorkspaceOutputs', 'on');                % second, silent run for the figure

%% 5. Sharp picture of the block diagram for the report ------------------------------------
try
    print(['-s' mdl], '-dpdf', 'fig2a_simulink_model.pdf');          % vector (sharpest)
    print(['-s' mdl], '-dpng', '-r300', 'fig2a_simulink_model.png');  % 300 dpi image (for Word)
catch err
    warning('Could not save the block-diagram picture: %s', err.message);
end

%% 6. One results figure (white background, same colours as the comparison figure) -----------
% read theta, x, F: from the logged signals, or (fallback) straight from the model outputs
try
    ts = @(n) out.logsout.get(n).Values;
    th = ts('theta');  xx = ts('x');  FF = ts('F');
catch
    warning('Logged signals not found; re-running with To Workspace blocks.');
    add_block('simulink/Sinks/To Workspace', [mdl '/log_theta'], 'VariableName', 'sl_theta', ...
              'SaveFormat', 'Timeseries', 'Position', [700 230 760 250]);
    add_block('simulink/Sinks/To Workspace', [mdl '/log_x'], 'VariableName', 'sl_x', ...
              'SaveFormat', 'Timeseries', 'Position', [700 270 760 290]);
    add_block('simulink/Sinks/To Workspace', [mdl '/log_F'], 'VariableName', 'sl_F', ...
              'SaveFormat', 'Timeseries', 'Position', [700 310 760 330]);
    add_line(mdl, 'Demux/3', 'log_theta/1', 'autorouting', 'smart');
    add_line(mdl, 'Demux/1', 'log_x/1',     'autorouting', 'smart');
    add_line(mdl, 'PID/1',   'log_F/1',     'autorouting', 'smart');
    out = sim(mdl, 'ReturnWorkspaceOutputs', 'on');
    th = out.sl_theta;  xx = out.sl_x;  FF = out.sl_F;
end
blue = [0 0.45 0.74];  green = [0.13 0.55 0.13];  red = [0.85 0.10 0.10];

fig = figure('Name', 'Simulink results', 'Units', 'centimeters', 'Position', [2 2 16 15], 'Color', 'w');
tl = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, 'Simulink model pendulum\_pid: closed-loop response, \theta_0 = 1.37^\circ', 'FontWeight', 'bold');

ax(1) = nexttile;  plot(th.Time, rad2deg(squeeze(th.Data)), 'Color', blue, 'LineWidth', 1.6);
yline(0, ':', 'Color', [0.5 0.5 0.5]);  grid on;  ylabel('\theta [deg]');  title('(a) Pendulum angle \theta');
ax(2) = nexttile;  plot(xx.Time, 1e3*squeeze(xx.Data), 'Color', green, 'LineWidth', 1.6);
grid on;  ylabel('x [mm]');  title('(b) Cart position x');
ax(3) = nexttile;  plot(FF.Time, squeeze(FF.Data), 'Color', red, 'LineWidth', 1.6);
yline(0, ':', 'Color', [0.5 0.5 0.5]);  grid on;  ylabel('F [N]');  xlabel('Time [s]');
title('(c) Control force F');
linkaxes(ax, 'x');  xlim([0 Tend]);
try, theme(fig, 'light'); catch, end                              % white even in dark-mode MATLAB
try, fig.WindowStyle = 'normal'; catch, end                       % own window, not docked
drawnow;  figure(fig);  shg;                                      % bring it IN FRONT of the scopes
exportgraphics(fig, 'fig_simulink_results.pdf', 'ContentType', 'vector');
exportgraphics(fig, 'fig_simulink_results.png', 'Resolution', 300, 'BackgroundColor', 'white');
pause(1);  figure(fig);                                           % again, after the scopes finish drawing

fprintf('Done: %s.slx built and simulated. Figures saved: fig2a_simulink_model.pdf/.png, fig_simulink_results.pdf/.png\n', mdl);

%% ======================================= local functions ==================================
function name_out(blk, port, name)               % label a wire and log the signal
    ph = get_param(blk, 'PortHandles');
    set_param(ph.Outport(port), 'Name', name, 'DataLogging', 'on');
end

function scope_setup(blk, ttl, ylab, ylims, pos)
    c = get_param(blk, 'ScopeConfiguration');
    c.TimeSpan = '8';   c.ShowGrid = true;   c.Position = pos;
    try, c.Title  = ttl;   catch, end
    try, c.YLabel = ylab;  catch, end
    try, c.YLimits = ylims; catch, end
end