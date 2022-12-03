% PADA Plant Model

m = 1; % Mass of aircraft [kg]
g = 9.81; % Gravitational acceleration [m/s^2]
rho = 1;

% Motor & Propulsion properties
S_prop = 1;
C_prop = 1;
k_motor = 1;

% Wing properties
Ref_area = 1;
Ref_span = 1;
Ref_length = 1;
Ref_chord = 1;

% Stability & Control derivatives
% Lift
C_L0 = 0;
C_Lalpha = 0;
C_Lde = 0;

% Drag
C_D0 = 0;
C_Dalpha = 0;
C_Dde = 0;

% Side force
C_y0 = 0;
C_ybeta = 0;
C_yda = 0;

% Roll moment
C_l0 = 0;
C_lbeta = 0;
C_lda = 0;

% Pitch moment
C_m0 = 0;
C_malpha = 0;
C_mde = 0;

% Yaw moment
C_n0 = 0;
C_nbeta = 0;
C_nda = 0;
