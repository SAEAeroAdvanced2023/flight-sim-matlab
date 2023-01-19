% PADA Plant Model

m = 0.4536; % Mass of aircraft [kg]
g = 9.81; % Gravitational acceleration [m/s^2]
rho = 1.225; % Air density [kg/m^3]

% Motor & Propulsion properties
S_prop = 0.25; %Guess [m^2]
k_motor = 20; %Guess

% Wing properties
Ref_area = 0.1039; % Wing area [m^2]
Ref_span = 0.4597; % Wing span [m]
Ref_chord = 0.2472; % Wing chord [m]

% Moments of inertia
J_x = 0.1; %Guess [kg m^2]
J_y = 0.05; %Guess [kg m^2]
J_z = 0.15; %Guess [kg m^2]
J_xz = 0.0015; %Guess [kg m^2]

Ref_length = 1; 
