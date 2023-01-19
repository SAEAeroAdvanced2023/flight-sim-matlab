% Stability & Control derivatives

% Lift coefficients in stability frame
C_L0 = 0.015; % Updated Jan 19 8:45am
C_Lalpha = 2.60667; % Updated Jan 19 8:45am
C_Lde = 0;
C_Lq = 3.3277; % Updated Jan 19 8:45am
% Drag coefficients in stability frame
C_D0 = 0.0139; % Updated Jan 19 8:45am
C_Dalpha = 0.2845; % Approx updated Jan 19 8:45am
C_Dde = 0;
C_Dq = 0;
% Sidesslip force coeffients in Y-axis
C_Y0 = 0; % Updated Jan 19 8:45am
C_Ybeta = -0.63965; % Updated Jan 19 8:45am
C_Yda = -0.02141; % Updated Jan 19 8:45am
C_Yp = -0.06953; % Updated Jan 19 8:45am
C_Yr = 0.51802; % Updated Jan 19 8:45am

% Control Derivatives
% Elevator
C_Xde = -0.00834; % Updated Jan19 10:30am
C_Zde = -0.88788; % Updated Jan19 10:30am
% Aileron
C_Xda = -0.00017; % Updated Jan19 10:30am
C_Zda = 0; % Updated Jan19 10:30am


% Roll moment coefficients
C_l0 = 0; % Updated Jan 19 8:45am
C_lbeta = -0.03987; % Updated Jan 19 8:45am
C_lda = 0.21662; % Updated Jan 19 8:45am
% Pitch moment coefficients
C_m0 = 0.00035; % Updated Jan 19 8:45am
C_malpha = -0.34517; % Updated Jan 19 8:45am
C_mde = -0.44195; % Updated Jan 19 8:45am
C_mq = -1.23417; % Updated Jan 19 8:45am
% Yaw moment coefficients
C_n0 = 0; % Updated Jan 19 8:45am
C_nbeta = 0.12715; % Updated Jan 19 8:45am
C_nda = 0.00012; % Updated Jan 19 8:45am