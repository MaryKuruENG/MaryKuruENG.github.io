% params.m -- analytical model, Simulink model and comparisons.
% All values come from the SolidWorks -> Simscape export (Pendulum-Assy_1.xml).
% Platform is fixed ground: its mass does not enter the dynamics, so it is not used.

%% ---- Pendulum (rod + bob + pivot pin) = ONE rigid body rotating about the pivot ----
m      = 6.6403614345;     % total pendulum mass [kg]  (steel, rho = 7800 kg/m^3)
zCoM   = 0.1339479566;     % CoM along rod axis from part origin [m]      (CAD)
zPivot = 0.5000000000;     % pivot-pin axis along same axis [m]           (CAD mate Concentric20)
l      = zPivot - zCoM;    % pivot -> CoM distance [m]                    (= 0.36605)
I_cm   = 0.2524117820;     % inertia about the pivot-axis direction THROUGH the CoM [kg m^2] (CAD, Ixx)
J      = I_cm + m*l^2;     % PARALLEL-AXIS THEOREM: inertia about the PIVOT [kg m^2] (= 1.14218)
I      = I_cm;             % (name used by the equations: I + m*l^2 = J)
% The bob is fixed to the rod (no spin relative to it), so no extra bob-spin term is added.

%% ---- Cart (body + axles) and 4 wheels -------------------------------------
m_cart = 71.4527431673;    % cart body incl. axles [kg]  (steel)
n_w    = 4;                % wheels
m_w    = 0.5881061448;     % one wheel [kg]
I_w    = 5.8810614475e-04; % one wheel, inertia about its axle [kg m^2]
R_w    = 0.04;             % wheel radius [m]
% Ideal rolling (no slip): each wheel adds translational mass + I_w/R^2 of equivalent mass
M = m_cart + n_w*(m_w + I_w/R_w^2);   % effective cart mass [kg] (= 75.2754)

%% ---- Assumptions -------------------------------------------------------------
b  = 0;        % cart viscous friction [N s/m]        -> NO friction (project assumption)
c  = 0;        % pivot viscous friction [N m s/rad]   -> NO friction (project assumption)
g  = 9.81;     % gravity [m/s^2]
th0 = asin(0.023848750116849705);   % initial tilt [rad] = 1.37 deg = tilt of the exported CAD pose

%% ---- Linearised model about theta = 0 (upright) --------------------------------
% states [x; xdot; theta; thetadot], input F [N]
D = (M+m)*J - (m*l)^2;
A = [0  1         0                  0;
     0 -J*b/D    -m^2*g*l^2/D        m*l*c/D;
     0  0         0                  1;
     0  m*l*b/D   (M+m)*m*g*l/D     -(M+m)*c/D];
B = [0; J/D; 0; -m*l/D];
C = [1 0 0 0; 0 0 1 0];  Dm = [0; 0];
