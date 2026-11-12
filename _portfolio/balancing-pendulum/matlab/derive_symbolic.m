% derive_symbolic.m -- Phase 1: Lagrangian derivation + linearisation (Symbolic Math Toolbox)
% x = cart position, th = pendulum angle from UPRIGHT. Cart friction b, pivot friction c.
clear; clc;
syms x th xd thd xdd thdd M m l I g b c F real
q = [x; th]; qd = [xd; thd];

% Kinetic energy: cart (incl. wheels, effective mass M) + pendulum (CoM translation + rotation)
pxd = xd + l*cos(th)*thd;  pyd = -l*sin(th)*thd;
T = 1/2*M*xd^2 + 1/2*m*(pxd^2 + pyd^2) + 1/2*I*thd^2;
V = m*g*l*cos(th);
Lag = simplify(T - V);

% Lagrange: d/dt(dL/dqd) - dL/dq = Q,  Q = [F - b*xd; -c*thd]
Q = [F - b*xd; -c*thd];
eqs = sym(zeros(2,1));
for k = 1:2
    dLdqd = diff(Lag, qd(k));
    ddt = diff(dLdqd,x)*xd + diff(dLdqd,th)*thd + diff(dLdqd,xd)*xdd + diff(dLdqd,thd)*thdd;
    eqs(k) = simplify(ddt - diff(Lag,q(k)) - Q(k));
end
disp('Nonlinear equations of motion (each = 0):'); pretty(eqs)

sol = solve(eqs,[xdd; thdd]);  xdd_nl = simplify(sol.xdd);  thdd_nl = simplify(sol.thdd);

% Linearise about th = thd = xd = 0, F = 0
f = [xd; xdd_nl; thd; thdd_nl];  s = [x; xd; th; thd];
Asym = simplify(subs(jacobian(f,s),[th thd xd],[0 0 0]));
Bsym = simplify(subs(jacobian(f,F),[th thd xd],[0 0 0]));
Csym = [1 0 0 0; 0 0 1 0];  Dsym = [0; 0];     % outputs: x and theta; no direct feedthrough
disp('A ='); pretty(Asym); disp('B ='); pretty(Bsym);
disp('C ='); disp(Csym); disp('D ='); disp(Dsym);

%% Numeric check against params.m  (should print ~0)
symList = {M,m,l,I,g,b,c};          % keep the symbols before params.m overwrites the names
params;
vals = {M,m,l,I,g,b,c};
Anum = double(subs(Asym,symList,vals));
Bnum = double(subs(Bsym,symList,vals));
fprintf('norm(A_symbolic - A_params) = %.2e\n', norm(Anum - A));
fprintf('norm(B_symbolic - B_params) = %.2e\n', norm(Bnum - B));
