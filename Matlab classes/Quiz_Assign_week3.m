%% Quiz and assignment week 3

%% Quiz part

clc
clear
close all

% Question 3

a1=0.8;
a2=a1;
q1=30;
q2=20;

trchain2('R(q1) Tx(a1) R(q2) Tx(a2)',[q1 q2])

q1=50;
q2=-20;

trchain2('R(q1) Tx(a1) R(q2) Tx(a2)',[q1 q2])


%% Programming part 1

clc
clear
close all

% DO NOT EDIT THE FOLLOWING LINES
a = [0.8533, 1.6935, 9.6046]; % accelerometer measurements in m/s2
b = [34.59, -11.36, 29.72]*1e-6; % magnetometer measurements in T
omega = [-0.12 0.04 0.27];  % gyroscope measurements in radians/sec

I = 40; % local magnetic inclination in degrees
B = 47.0e-6;  % local magnetic field strength in T
g = 9.79; % local gravitational acceleration in m/s2
        
% YOUR ANSWER BELOW HERE



roll = atand(a(2)/a(3))
pitch = asind(-a(1)/g)
yaw = atand((cosd(pitch)*(b(3)*sind(roll)-b(2)*cosd(roll)))/(b(1)+B*sind(I)*sind(pitch)))

%% Programming part 2 

clc
clear
close all

% DO NOT EDIT THE FOLLOWING LINES
R  = rpy2r(20, 30, -40, 'deg');
omega = [-0.12 0.04 0.27];  % gyroscope measurements in radians/sec
        
% YOUR ANSWER BELOW HERE

S = [
    0 -omega(3) omega(2)
    omega(3) 0 -omega(1)
    -omega(2) omega(1) 0 
    ];

Rdot = S*R


RPY_T = tr2rpy(trnorm(R+50*1e-3*Rdot))




