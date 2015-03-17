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

%% Programming part 3 

clc
clear
close all

% DO NOT EDIT THE FOLLOWING LINES
q = [30 -40 10];  % joint angles in degrees

% YOUR ANSWER BELOW HERE

a1=9;
a2=3;
a3=7;
a4=-3;
a5=5.1;

q1 = deg2rad(q(1));
q2 = deg2rad(q(2)); 
q3 = deg2rad(q(3));

T=trchain2('R(q1) Tx(a1) Ty(a2) R(q2) Tx(a3) R(q3) Ty(a4) Tx(a5)',[q1 q2 q3]);

TM = trot2(q(1), 'deg') * transl2(9.0,3.0) * trot2(q(2), 'deg') * transl2(7.0,0) * trot2(q(3), 'deg') * transl2(5.1,-3);%Equivalent

x = TM(1,3)*1e-2
y = TM(2,3)*1e-2
theta = acosd(T(1,1))

%% DH example (not sure if it works

clc
clear
close all

q = [30 -40 10];

qrad=q.*pi/180

a1=9*10^(-2)
a2=7*10^(-2)
a3=5.1*10^(-2)
s1 = 3*10^(-2)
% initial values for DH coordinate system
q1f=atan(s1/a1)
q2f=-atan(s1/a1)
q3f=-(pi/2-atan(a3/s1))
q4f=(pi/2-atan(a3/s1))

L(1) = Link([0 0 sqrt(a1^2+s1^2) 0]);
L(2) = Link([0 0 a2 0]);
L(3) = Link([0 0 sqrt(s1^2+a3^2) 0]);
L(4) = Link([0 0 0 0]); % only for transforming to a frame beeing perpendicular to end effector
three_link = SerialLink(L, 'name', 'three link');

three_link.plot

%% Programming part 4 

clc
clear
close all

% DO NOT EDIT THE FOLLOWING LINES
q1 = [-30 20 10 30 40 50];  % joint angles in degrees
q2 = [30 40 20 10 20 80];   % joint angles in degrees
        
% YOUR ANSWER BELOW HERE
mdl_puma560

p560.base = transl(17.4,12.2,1.3) * trotz(pi/2);
p560.tool = transl(4.1*1e-2,0.3*1e-2,17.4*1e-2);

qt=jtraj(q1,q2,120); % Define the trajectory 

P70r= p560.fkine(qt(70,:),'deg') % Calculates the transformation matrice @ 70

P70 = P70r(1:3,4)' % Returns only the position
