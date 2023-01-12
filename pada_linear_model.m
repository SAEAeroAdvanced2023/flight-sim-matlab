run("pada_plant.m")

u_0 = 0;
w_0 = 0;
q_0 = 0;
de_0 = 0;
dt_0 = 0;
Va_0 = 1;
theta_0 = 0;
alpha_0 = 0;

J_y = 1;

% Force coefficients in body X-axis
C_X0 = -C_D0*cos(alpha_0) + C_L0*sin(alpha_0);
C_Xalpha = -C_Dalpha*cos(alpha_0) + C_Lalpha*sin(alpha_0);
C_Xde = -C_Dde*cos(alpha_0) + C_Lde*sin(alpha_0);
C_Xq = 0;

% State u coefficients
X_u = (u_0*rho*Ref_area/m)*(C_X0 + C_Xalpha*alpha_0 + C_Xde*de_0) - (rho*Ref_area*w_0*C_Xalpha/(2*m)) + (rho*Ref_area*Ref_chord*C_Xq*u_0*q_0/(4*m*Va_0)) - (rho*S_prop*C_prop*u_0/m);
X_w = -q_0 + (w_0*rho*Ref_area/m)*(C_X0 + C_Xalpha*alpha_0 + C_Xde*de_0) + (rho*Ref_area*Ref_chord*C_Xq*w_0*q_0/(4*m*Va_0)) + (rho*Ref_area*C_Xalpha*u_0/(2*m)) - (rho*S_prop*C_prop*w_0/m);
X_q = -w_0 + (rho*Va_0*Ref_area*C_Xq*Ref_chord/(4*m));
X_de = rho*Va_0*Va_0*Ref_area*C_Xde/(2*m);
X_dt = rho*S_prop*C_prop*k_motor*k_motor*dt_0/m;

C_Z0 = 0;
C_Zalpha = 0;
C_Zde = 0;
C_Zq = 0;

Z_u = q_0 + (u_0*rho*Ref_area/m)*(C_Z0 + C_Zalpha*alpha_0 + C_Zde*de_0) - (rho*Ref_area*C_Zalpha*w_0/(2*m)) + (u_0*rho*Ref_area*C_Zq*Ref_chord*q_0/(4*m*Va_0));
Z_w = (w_0*rho*Ref_area/m)*(C_Z0 + C_Zalpha*alpha_0 + C_Zde*de_0) + (rho*Ref_area*C_Zalpha*u_0/(2*m)) + (rho*w_0*Ref_area*Ref_chord*C_Zq*q_0/(4*m*Va_0));
Z_q = u_0 + (rho*Va_0*Ref_area*C_Zq*Ref_chord/(4*m));
Z_de = rho*Va_0*Va_0*Ref_area*C_Zde/(2*m);

C_m0 = 0;
C_malpha = 0;
C_mde = 0;
C_mq = 0;

M_u = (u_0*rho*Ref_area*Ref_chord/J_y)*(C_m0 + C_malpha*alpha_0 + C_mde*de_0) - (rho*Ref_area*Ref_chord*C_malpha*w_0/(2*J_y)) + (rho*Ref_area*Ref_chord*Ref_chord*C_mq*q_0*u_0/(4*J_y*Va_0));
M_w = (w_0*rho*Ref_area*Ref_chord/J_y)*(C_m0 + C_malpha*alpha_0 + C_mde*de_0) + (rho*Ref_area*Ref_chord*C_malpha*u_0/(2*J_y)) + (rho*Ref_area*Ref_chord*Ref_chord*C_mq*q_0*w_0/(4*J_y*Va_0));
M_q = rho*Va_0*Ref_area*Ref_chord*Ref_chord*C_mq/(4*J_y);
M_de = rho*Va_0*Va_0*Ref_area*Ref_chord*C_mde/(2*J_y);

A_lon = [X_u, X_w, X_q+w_0, -g*cos(theta_0);
         Z_u, Z_w, Z_q-w_0, -g*sin(theta_0);
         M_u, M_w, M_q, 0;
         0, 0, 1, 0]

B_lon = [X_de, X_dt;
         Z_de, 0;
         M_de, 0;
         0, 0]