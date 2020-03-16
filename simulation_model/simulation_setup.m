
%run this file to set up everything for the matlab simulink file
%% Define the nescessary constants

%define the parameters of the model
M = 1; %mass of UAV
m = 0.5; %mass of payload
L = 0.5; %Lenght of cable
b = 0.15; %motor distance from centre of mass
J = (1/12)*M*(2*b)^2;  %quadrotor moment of inertia
g = 9.81; %gravitational constant
theta_max = pi/4; %maximum tilt angle

%motor properties
fmax = 15; %maximum motor thrust
fmin = fmax*0.1; %minimum motor thrust
motor_t_const = 1/200; %time constant of the motor

%define the drag properties of the model
rho = 1.225; %rho, air density
Cd = 1; %drag coeficient
area = 0.1; %quadrotor surface area
ba = 0.001; %damping on pendulum

%define the sensor noise
noise_p_pos = 0.00001;
noise_freq_pos = 0.001;

noise_p_vel = 0.00001;
noise_freq_vel = 0.001;

noise_p_theta = 0.00001;
noise_freq_theta = 0.001;
noise_p_theta_dot = 0.00001;
noise_freq_theta_dot = 0.001;


drag = 1;

%% Angle controller
%desired pole locations angle control
zeta = 0.707; % desired damping 
ts = 0.15; %desired settling time

%calculate the matching settling time
wn = 4 / (ts * zeta); % desired natural frequency
zeta = 1/sqrt(2); % desired damping 

% desired position of CL poles
p = [-zeta*wn+1i*sqrt(1-zeta^2)*wn, -zeta*wn-1i*sqrt(1-zeta^2)*wn]; 
omega = zeta*wn;
wd = sqrt(1-zeta^2)*wn;

%Calculate gain for inner loop
k1 = (2 * omega)*J;

%Calulate gain for outer loop
s = p(1);
a = k1/J;
k2 = abs(( s * (s + a) ) / a);

%% Vertual velocity controller
%this is the velocity controller used by the planner
kxprop = 7;
kzprop = 15;

%% Horizontal State Regulator
%implement x ss variables
A_x = [0 1; 0 0];
b_x = [0 ; 1/M];
c_x = [1 0];

%desired x pole locations
zeta_x = 1/sqrt(2);
ts_x = 1;

wn_x = 4 / (ts_x * zeta_x); % desired natural frequency
zeta_x = 1/sqrt(2); % desired damping 
p_x = [-zeta_x*wn_x+1i*sqrt(1-zeta_x^2)*wn_x, -zeta_x*wn_x-1i*sqrt(1-zeta_x^2)*wn_x]; % desired CL poles

k_x = acker(A_x,b_x,p_x);

%% Vertical State Regulator
%implement z ss variables
A_z = [0 1; 0 0];
b_z = [0 ; (1/M)];
c_z = [1 0];

%desired z pole locations
zeta_z = 1/sqrt(2); % desired damping 
ts_z = 1;


wn_z = 4 / (ts_z * zeta_z); % desired natural frequency
zeta_z = 1/sqrt(2); % desired damping 
p_z = [-zeta_z*wn_z+i*sqrt(1-zeta_z^2)*wn_z, -zeta_z*wn_z-i*sqrt(1-zeta_z^2)*wn_z]; % desired CL poles

k_z = acker(A_z,b_z,p_z);


%% Load the planned input sequence

%Input for complex action sequence
tun_chim_tun__obs_input = load ('../example_trajectories/complex_action_sec.mat');
tun_chim_tun_obs.time = tun_chim_tun__obs_input.t';
tun_chim_tun_obs.signals.values = [tun_chim_tun__obs_input.rz',tun_chim_tun__obs_input.rx',tun_chim_tun__obs_input.input_type'];
tun_chim_tun_obs.signals.dimensions =3;

%obtain the planning sampling time
ts_planning = tun_chim_tun__obs_input.t(2) - tun_chim_tun__obs_input.t(1);

%get the duration of execution
execution_time = tun_chim_tun__obs_input.t(end) + 1;

%% Simmulate the ideal input sequence
%this step is performed to convert the input sequence to force references 

sim('get_force_path');

path_fz_dat = fz.Data;
path_fx_dat = fx.Data;
path_a_dat = path_a.Data;
path_a_dot_dat = path_a_dot.Data;
path_x_dot_dat = path_x_dot.Data;
path_x_pos_dat = path_x_pos.Data;
path_z_dot_dat = path_z_dot.Data;
path_z_pos_dat= path_z_pos.Data;


%save('matlab_outputs/expected_trajectory.mat','path_fz_dat','path_fx_dat','path_a_dat','path_a_dot_dat','path_x_dot_dat','path_x_pos_dat','path_z_dot_dat','path_z_pos_dat','tout','-v7')
%% Save Stuffs
%run simmulink model
%sim('ss_control_system')

%save to file, use older version to be able to read it with python
%save('angle_control.mat','f1','f2','theta','desired','ref','tout','-v7')
%save('ss.mat','z','x','alpha','theta','L','tout','-v7')
%save('ss_x.mat','x','-v7')
%save('ss_z.mat','z','-v7')
