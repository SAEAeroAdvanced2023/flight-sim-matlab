% The following script creates the longitudinal and lateral state space
% models for the PADA using the theory presented in Small Unmanned Aircraft
% : Theory and Practice (https://concordiauniversity.on.worldcat.org/oclc/769927217)

% Load PADA variables
clear;
run("pada_plant.m");
run("pada_aero_coefficients.m");

% ---------------------------------
% Trim conditions for linearization
% ---------------------------------

u_0 = 0; % Velocity in body x-axis
v_0 = 0; % Velocity in body y-axis
w_0 = 0; % Velocity in body z-axis
Va_0 = 1; % Airspeed

p_0 = 0; % Angular velocity in x-axis
q_0 = 0; % Angular velocity in y-axis
r_0 = 0; % Angular velocity in z-axis

de_0 = 0; % Elevator deflection
da_0 = 0; % Aileron deflection
dt_0 = 0; % Throttle deflection

theta_0 = 0; % Pitch angle
phi_0 = 0; % Roll angle
alpha_0 = 0; % Angle of attack
beta_0 = 0; % Sideslip angle


% -------------------------
% Lateral state-space model
% -------------------------

% State coefficients for delta v
Y_v = (rho*Ref_area*v_0/(4*m*Va_0))*(C_Yp*p_0 + C_Yr*r_0) + (rho*Ref_area*v_0/m)*(C_Y0 + C_Ybeta*beta_0 + C_Yda*da_0) + (rho*Ref_area*C_Ybeta/(2*m))*sqrt(u_0*u_0 + w_0*w_0);
Y_p = w_0 + (rho*Va_0*Ref_area*Ref_span*C_Yp/(4*m));
Y_r = -u_0 + (rho*Va_0*Ref_area*Ref_span*C_Yr/(4*m));
Y_da = (rho*Va_0*Va_0*Ref_area*C_Yda/(2*m));

% Moment of inertia variables
temp = J_x*J_z - J_xz*J_xz;
temp1 = J_xz*(J_x - J_y + J_z)/temp;
temp2 = (J_z*J_z - J_z*J_y + J_xz*J_xz)/temp;
temp7 = (J_x*J_x - J_x*J_y + J_xz*J_xz)/temp;

% State coefficients for delta p
L_v = (rho*Ref_area*Ref_span*Ref_span*v_0/(4*Va_0))*(C_pp*p_0 + C_pr*r_0) + (rho*Ref_area*Ref_span*v_0)*(C_p0 + C_pbeta*beta_0 + C_pda*da_0) + (rho*Ref_area*Ref_span*C_pbeta/2)*sqrt(u_0*u_0 + w_0*w_0);
L_p = (temp1*q_0) + (rho*Va_0*Ref_area*Ref_span*Ref_span*C_pp/4);
L_r = (-temp2*q_0) + (rho*Va_0*Ref_area*Ref_span*Ref_span*C_pr/4);
L_da = (rho*Va_0*Va_0*Ref_area*Ref_span*C_pda/2);

% State coefficients for delta r
N_v = (rho*Ref_area*Ref_span*Ref_span*v_0/(4*Va_0))*(C_rp*p_0 + C_rr*r_0) + (rho*Ref_area*Ref_span*v_0)*(C_r0 + C_rbeta*beta_0 + C_rda*da_0) + (rho*Ref_area*Ref_span*C_rbeta/2)*sqrt(u_0*u_0 + w_0*w_0);
N_p = (temp7*q_0) + (rho*Va_0*Ref_area*Ref_span*Ref_span*C_rp/4);
N_r = (-temp1*q_0) + (rho*Va_0*Ref_area*Ref_span*Ref_span*C_rr/4);
N_da = (rho*Va_0*Va_0*Ref_area*Ref_span*C_rda/2);


A_lat = [Y_v,                   Y_p/(Va_0*cos(beta_0)), Y_r/(Va_0*cos(beta_0)),     (g*cos(theta_0)*cos(phi_0)/(Va_0*cos(beta_0))),             0;
         L_v*Va_0*cos(beta_0),  L_p,                    L_r,                        0,                                                          0;
         N_v*Va_0*cos(beta_0),  N_p,                    N_r,                        0,                                                          0;
         0,                     1,                      cos(phi_0)*tan(theta_0),    q_0*cos(phi_0)*tan(theta_0) - r_0*sin(phi_0)*tan(theta_0),  0;
         0,                     0,                      cos(phi_0)*sec(theta_0),    p_0*cos(phi_0)*sec(theta_0) - r_0*sin(phi_0)*sec(theta_0),  0]

B_lat = [Y_da/(Va_0*cos(beta_0)); 
         L_da; 
         N_da; 
         0; 
         0]


% ------------------------------
% Longitudinal state-space model
% ------------------------------

% Force coefficients in body X-axis
C_X0 = -C_D0*cos(alpha_0) + C_L0*sin(alpha_0);
C_Xalpha = -C_Dalpha*cos(alpha_0) + C_Lalpha*sin(alpha_0);
C_Xde = -C_Dde*cos(alpha_0) + C_Lde*sin(alpha_0);
C_Xq = -C_Dq*cos(alpha_0) + C_Lq*sin(alpha_0);
% State coefficients for delta u
X_u = (u_0*rho*Ref_area/m)*(C_X0 + C_Xalpha*alpha_0 + C_Xde*de_0) - (rho*Ref_area*w_0*C_Xalpha/(2*m)) + (rho*Ref_area*Ref_chord*C_Xq*u_0*q_0/(4*m*Va_0)) - (rho*S_prop*C_prop*u_0/m);
X_w = -q_0 + (w_0*rho*Ref_area/m)*(C_X0 + C_Xalpha*alpha_0 + C_Xde*de_0) + (rho*Ref_area*Ref_chord*C_Xq*w_0*q_0/(4*m*Va_0)) + (rho*Ref_area*C_Xalpha*u_0/(2*m)) - (rho*S_prop*C_prop*w_0/m);
X_q = -w_0 + (rho*Va_0*Ref_area*C_Xq*Ref_chord/(4*m));
X_de = rho*Va_0*Va_0*Ref_area*C_Xde/(2*m);
X_dt = rho*S_prop*C_prop*k_motor*k_motor*dt_0/m;

% Force coefficients in body Z-axis
C_Z0 = -C_D0*sin(alpha_0) - C_L0*cos(alpha_0);
C_Zalpha = -C_Dalpha*sin(alpha_0) - C_Lalpha*cos(alpha_0);
C_Zde = -C_Dde*sin(alpha_0) - C_Lde*cos(alpha_0);
C_Zq = -C_Dq*sin(alpha_0) - C_Lq*cos(alpha_0);
% State coefficients for delta w
Z_u = q_0 + (u_0*rho*Ref_area/m)*(C_Z0 + C_Zalpha*alpha_0 + C_Zde*de_0) - (rho*Ref_area*C_Zalpha*w_0/(2*m)) + (u_0*rho*Ref_area*C_Zq*Ref_chord*q_0/(4*m*Va_0));
Z_w = (w_0*rho*Ref_area/m)*(C_Z0 + C_Zalpha*alpha_0 + C_Zde*de_0) + (rho*Ref_area*C_Zalpha*u_0/(2*m)) + (rho*w_0*Ref_area*Ref_chord*C_Zq*q_0/(4*m*Va_0));
Z_q = u_0 + (rho*Va_0*Ref_area*C_Zq*Ref_chord/(4*m));
Z_de = rho*Va_0*Va_0*Ref_area*C_Zde/(2*m);

% State coefficients for delta q
M_u = (u_0*rho*Ref_area*Ref_chord/J_y)*(C_m0 + C_malpha*alpha_0 + C_mde*de_0) - (rho*Ref_area*Ref_chord*C_malpha*w_0/(2*J_y)) + (rho*Ref_area*Ref_chord*Ref_chord*C_mq*q_0*u_0/(4*J_y*Va_0));
M_w = (w_0*rho*Ref_area*Ref_chord/J_y)*(C_m0 + C_malpha*alpha_0 + C_mde*de_0) + (rho*Ref_area*Ref_chord*C_malpha*u_0/(2*J_y)) + (rho*Ref_area*Ref_chord*Ref_chord*C_mq*q_0*w_0/(4*J_y*Va_0));
M_q = rho*Va_0*Ref_area*Ref_chord*Ref_chord*C_mq/(4*J_y);
M_de = rho*Va_0*Va_0*Ref_area*Ref_chord*C_mde/(2*J_y);

A_lon = [X_u,          X_w,           X_q + w_0, -g*cos(theta_0),                       0;
         Z_u,          Z_w,           Z_q - w_0, -g*sin(theta_0),                       0;
         M_u,          M_w,           M_q,       0,                                     0;
         0,            0,             1,         0,                                     0;
         sin(theta_0), -cos(theta_0), 0,         (u_0*cos(theta_0) + w_0*sin(theta_0)), 0]

B_lon = [X_de,  X_dt;
         Z_de,  0;
         M_de,  0;
         0,     0;
         0,     0]

% ---------------------
% LQR Controller Design
% ---------------------

% Laterial controller
Q_lat = eye(5);
R_lat = 5000; % Temp variable

[K_lat, S_lat, P_lat] = lqr(A_lat, B_lat, Q_lat, R_lat);

% Longitudinal controller
Q_lon = eye(5);
R_lon = [5, 0; 0, 0.1]; % Temp variables

[K_lon, S_lon, P_lon] = lqr(A_lon, B_lon, Q_lon, R_lon);

