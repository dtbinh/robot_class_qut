%% Lecture 4 matlab code

%% 4.1 Paths and trajectories

clc
clear all
close all

T0=eye(4) % First frame

T1=transl(1,2,3)*rpy2tr(0.6,0.8,1.4) % Second frame

%trplot(T0)
%trplot(T1)
tranimate(T0,T1)

%% 4.2 Polynomial trajectory

clc
clear all
close all

% We want to reduce the acceleration peak to reduce the size of motors and
% reduce vibration => smooth (pos,vel,acc,jerk continuous) 
% In rob we use polynomial as smooth functions (5th order qintic)

% We use 6x6 matrices

tpoly(0,1,50) %we observe the time steps to get from 0 to 1
[s,sd,sdd] = tpoly(0,1,50); %We can store pos,vel,acc

figure
tpoly(0,1,50,0.5,0) % We can overwrite the init and final velocity but it changes the equation a lot (big overshoot)

% Finally not really efficient because the average speed is only 50 % of
% the maximum speed of the motor => wasting performance

% Exercise
close all
tpoly(-1,1,100)
figure
tpoly(-1,1,100,0.1)
figure
tpoly(-1,1,100,0.5)
s=tpoly(-1,1,100,0.5);
max(s)

%% 4.3 1D Trapezoidale trajectory

clc
clear all
close all

% Much quicker than the polnomiale but acceleration is not continuous but
% in general in works

lspb(0,1,50)
[s,sd,sdd] = lspb(0,1,50);

lspb(0,1,50,0.025) % Force the linear velocity (can be too small => error)

%Exercice :
close all
[s,sd,sdd] = tpoly(0,1,95);
max(sd)

tpoly(0,1,95);
figure
lspb(0,1,55,0.02); % Equivalent to tpoly at 95

%% 4.4 1D trajectory with via points

clc
clear all
close all

%Introduce blends between the points

first = 10;
last = 30;
via = [40;10;last];

mstraj(via,1,[],first,0.1,2)
figure
mstraj(via,1,[],first,0.1,4) % the more we increase the acc time the farther we are from the via pts
figure
mstraj(via,1,[],first,0.1,8)
figure
mstraj(via,1,[],first,0.1,0) %If we put 0 in acc time we pass on the via point but traj not smooth
pause;

close all
mstraj(via,1,[],first,0.1,4)
figure
mstraj(via,2,[],first,0.1,4) % If we increase velocity we execute the traj faster but farther from the points again
pause;

close all
mstraj(via,[],[10 30 20],first,0.1,4) % If we don't care about velocity but we want to be at one point at a certain time
pause;

close all;

%% 4.5 Multidimensional smooth trajectory

clc
clear all
close all

% We use linear interpolation

first = [10 20];% Let's create two 2D points in vector form
last = [30 10];

[x,xd] = jtraj(first,last,50); % joint interpolated trajectory
plot(x)
pause;
figure
plot(xd)
pause;
close all

[x,xd] = jtraj(first,last,50,[0 0], [10 10]); % We can set the init and final velocity
plot(x)
pause;
figure
plot(xd)
pause;
close all

% If we have 4 via points we need to introduce blends

start = [40 50];
via = [60 30; 40 10; 20 30; start];
x = mstraj(via,2, [],start,0.1,1); % 435 rows => 43.5s @ 2 unit/sec
plot(x)% show the position of x and y with time
pause;
figure
plot(x(:,1),x(:,2))% Show x from y (trajectory)
pause;
close all

x = mstraj(via, [1 3],[],start,0.1,1); % 835 rows => 83.5s twice as long becaus eslowed to 1 unit/s
plot(x)% show the position of x and y with time
pause;
figure
plot(x(:,1),x(:,2))% Show x from y (trajectory)
pause;
close all

%% 4.6 Rotation interpolation
clc
clear all
close all

x = jtraj([0,0,0],[-pi/2,pi/2,pi/4],100); % We create a trajectory between the two rpy frames
plot(x)
pause;

R = rpy2r(x); % R is 100 times a 3x3 matrix
%tranimate(R)
pause;
close

% Quaternion interpolation

q1 = Quaternion; %No rotation quaternion
q2 = Quaternion(rotx(pi/2)); % Rotation of pi/2 on x

q1.interp(q2,0)%Second argument is the position along the trajectory, 0 is first point
q1.interp(q2,1)% 1 is last point
q1.interp(q2,0.5)% 0.5 is mid-way between

ans.R % We can turn it back into a rotation matrix

% Quaternion interpolation is the best (better than rpy or euler zyz)

%% 4.7 Cartesian interpolation

% = interpolation of poses (translation + rotation)

% Translation : linear interpolation
% Rotation : Quaternion interpolation

clc
clear all
close all

T0 = eye(4);

T1 = transl(1,2,3)*rpy2tr(0.6,0.8,1.4);

T = ctraj(T0,T1,50);

%% Summary

% * Path = from A to B
% * Trajectory = Path + schedule
% * Smoothless : Continuous as time, to reduce motor
% * Typically we use polynomial order 5 but trapezoidal better but not
%   continuous in acceleration
% * Via-points : blends (acceleration period) but never actually get to the
%   point (very closed)
% * Interpolation : -Linear for translation
%                   -Quaternion for rotation

