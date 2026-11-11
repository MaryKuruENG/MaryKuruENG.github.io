%% Self-balancing inverted pendulum on a cart -- Phase 1 (model) + Phase 2 (PID)
% Parameters from CAD (params.m). No friction. PID on the pendulum angle.
clear; clc; close all;
params;

%% ---- Linearised model: state-space and transfer function ------------------
sys = ss(A,B,C,Dm,'StateName',{'x','xdot','theta','thetadot'}, ...
         'InputName','F','OutputName',{'x','theta'});
disp('Open-loop poles:'); disp(eig(A).');         % one positive pole -> unstable
G = tf(sys);  G_theta = minreal(G(2));  G_x = minreal(G(1));
disp('theta/F transfer function:'); G_theta
fprintf('Unstable pole p = %.4f rad/s\n',sqrt((M+m)*m*g*l/D));
% pidTuner(G_theta,'PID')    % <- interactive PID Tuner on the same plant

%% ---- PID gains (optimised: see optimize_pid.m) ----------------------------
% theta/F has NEGATIVE high-frequency gain, so the gains are negative (correct).
% Criterion: minimise ITAE of theta, subject to phase margin 60 deg, peak sensitivity
% Ms <= 1.2 and peak force <= 100 N for the CAD tilt (nominal actuator budget -> ASSUMED).
Kp = -4192.9;  Ki = -8365.6;  Kd = -645.6;
Tf = 1/100;                                       % derivative filter time constant (N = 100)

C_pid = pid(Kp,Ki,Kd,Tf);
L  = C_pid*G_theta;
CL = feedback(L,1);
assert(all(real(pole(CL)) < 0),'Closed loop unstable: check gains');
mg = allmargin(L);
fprintf('Gains: Kp=%.1f Ki=%.1f Kd=%.1f Tf=%.3f\n',Kp,Ki,Kd,Tf);
fprintf('Phase margin = %.1f deg at %.1f rad/s | peak sensitivity Ms = %.3f\n', ...
        min(mg.PhaseMargin), mg.PMFrequency(1), norm(feedback(1,L),inf));
save('pid_gains.mat','Kp','Ki','Kd','Tf');       % read by the Simulink and Simscape models

% Cross-check with the PID Tuner algorithm at the same crossover / phase margin
try
    Cchk = pidtune(G_theta,'PID',mg.PMFrequency(1),pidtuneOptions('PhaseMargin',60));
    fprintf('pidtune cross-check: Kp=%.1f Ki=%.1f Kd=%.1f Tf=%.4f\n',Cchk.Kp,Cchk.Ki,Cchk.Kd,Cchk.Tf);
catch
    disp('pidtune cross-check skipped');
end

%% ---- Simulations (initial tilt th0, cart at rest) ---------------------------
Tend = 8;  N = 1/Tf;
p  = [M m b l I g Kp Ki Kd N c];
s0 = [0; 0; th0; 0; 0; -th0];                     % [x xd th thd int filt]; filter pre-loaded (no kick)
opt = odeset('RelTol',1e-9,'AbsTol',1e-12,'MaxStep',1e-3);
[tn,sn] = ode45(@(t,s) dyn(t,s,p),[0 Tend],s0,opt);                           % nonlinear
[tl,sl] = ode45(@(t,s) dyn_lin(t,s,[A(:)' B' Kp Ki Kd N]),[0 Tend],s0,opt);   % linearised
Fn = zeros(size(tn));
for k = 1:numel(tn), [~,Fn(k)] = dyn(tn(k),sn(k,:)',p); end

%% ---- Plots -------------------------------------------------------------------
%% ---- One figure with all results (theta, x, F) ---------------------------------
fig = figure('Name','Closed-loop response','Units','centimeters','Position',[2 2 15 15]);
tl_ = tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
title(tl_,'Closed-loop response with PID control','FontWeight','bold');

% (a) Pendulum angle
ax1 = nexttile;
plot(tn, rad2deg(sn(:,3)), 'b',   'LineWidth',1.5); hold on;
plot(tl, rad2deg(sl(:,3)), 'r--', 'LineWidth',1.5);
yline(0,'k:');
ylabel('\theta [deg]'); grid on;
legend('Nonlinear','Linearised','Location','northeast');
title('(a) Pendulum angle \theta(t)');

% (b) Cart position
ax2 = nexttile;
plot(tn, sn(:,1)*1000, 'g',   'LineWidth',1.5); hold on;
plot(tl, sl(:,1)*1000, 'y--', 'LineWidth',1.5);
ylabel('x [mm]'); grid on;
legend('Nonlinear','Linearised','Location','southeast');
title('(b) Cart position x(t)');

% (c) Control force
ax3 = nexttile;
plot(tn, Fn, 'b', 'LineWidth',1.5);
yline(0,'k:');
ylabel('F [N]'); xlabel('Time [s]'); grid on;
title('(c) Control force F(t)');

%% ---- Numbers for the report ----------------------------------------------------
idx = find(abs(sn(:,3))>0.02*th0,1,'last');
fprintf('Settling time (2%%) of theta : %.2f s\n',tn(min(idx+1,end)));
fprintf('Peak undershoot             : %.1f %% of initial tilt\n',-100*min(sn(:,3))/th0);
fprintf('Peak control force          : %.1f N\n',max(abs(Fn)));
fprintf('Cart peak / final position  : %.1f mm / %.1f mm\n',1000*max(abs(sn(:,1))),1000*sn(end,1));
fprintf('Final cart velocity         : %.2e m/s | final theta %.2e rad\n',sn(end,2),sn(end,3));
fprintf('Max |theta_nl - theta_lin|  : %.2e rad\n',max(abs(sn(:,3)-interp1(tl,sl(:,3),tn))));
save('analytical_results.mat','tn','sn','tl','sl','Fn');   % compare with Simscape later

%% ---- local functions -----------------------------------------------------------
function [ds,F] = dyn(~,s,p)
    [M,m,b,l,I,g,Kp,Ki,Kd,N,c] = deal(p(1),p(2),p(3),p(4),p(5),p(6),p(7),p(8),p(9),p(10),p(11));
    xd=s(2); th=s(3); thd=s(4); xi=s(5); df=s(6);
    e = -th;
    F = Kp*e + Ki*xi + Kd*N*(e-df);                % PID, filtered derivative
    J = I + m*l^2;                                  % inertia about the pivot
    Mm  = [M+m, m*l*cos(th); m*l*cos(th), J];
    rhs = [F - b*xd + m*l*thd^2*sin(th); m*g*l*sin(th) - c*thd];
    acc = Mm\rhs;
    ds = [xd; acc(1); thd; acc(2); e; N*(e-df)];
end

function ds = dyn_lin(~,s,p)
    A = reshape(p(1:16),4,4); B = p(17:20)'; [Kp,Ki,Kd,N] = deal(p(21),p(22),p(23),p(24));
    e = -s(3); F = Kp*e + Ki*s(5) + Kd*N*(e-s(6));
    ds = [A*s(1:4) + B*F; e; N*(e-s(6))];
end
