%% optimize_pid.m -- reproduces the PID optimisation (optional, takes a few minutes)
% Minimise ITAE of theta (linearised closed loop, unit tilt) subject to
%   phase margin >= 60 deg, peak sensitivity Ms <= 1.2, peak force <= Fbudget for tilt th0.
clear; clc; params;
Tf = 1/100;  N = 1/Tf;
Gth = minreal(tf(ss(A,B,C(2,:),0)));              % theta/F
Fbudget = 100;  Msmax = 1.2;  PMmin = 60;          % Fbudget [N] is an ASSUMED actuator budget
t  = (0:0.004:12)';
x0 = [0;0;1;0;0;-1];                               % unit tilt, filter pre-loaded
v0 = log([4200 8400 650]);                         % start (magnitudes of Kp Ki Kd)
cost = @(v) pid_cost(v,A,B,Gth,N,Tf,t,x0,th0,Fbudget,Msmax,PMmin);
v  = fminsearch(cost,v0,optimset('Display','iter','TolX',1e-4,'TolFun',1e-8,'MaxIter',800));
K  = -exp(v);
fprintf('\nOptimised gains: Kp = %.1f, Ki = %.1f, Kd = %.1f\n',K(1),K(2),K(3));

function J = pid_cost(v,A,B,Gth,N,Tf,t,x0,th0,Fb,Msmax,PMmin)
    K = -exp(v); Kp = K(1); Ki = K(2); Kd = K(3);
    Acl = zeros(6); Acl(1:4,1:4) = A;
    Acl(1:4,3) = Acl(1:4,3) + B*(-Kp - Kd*N);
    Acl(1:4,5) = B*Ki;  Acl(1:4,6) = B*(-Kd*N);
    Acl(5,3) = -1;  Acl(6,3) = -N;  Acl(6,6) = -N;
    if max(real(eig(Acl))) > 1e-7, J = 1e6; return; end
    Cout = [0 0 1 0 0 0; 0 0 (-Kp-Kd*N) 0 Ki (-Kd*N)];     % outputs [theta; F]
    y  = initial(ss(Acl,zeros(6,1),Cout,[0;0]),x0,t);
    th = y(:,1);  F = y(:,2);
    itae = trapz(t,t.*abs(th));
    L  = pid(Kp,Ki,Kd,Tf)*Gth;
    Ms = norm(feedback(1,L),inf);
    mg = allmargin(L);  pm = min([mg.PhaseMargin 0 + 1e3]);
    J  = itae + 1e3*max(0,Ms-Msmax) + 1e3*max(0,max(abs(F))*th0-Fb)/Fb + 10*max(0,PMmin-pm);
end
