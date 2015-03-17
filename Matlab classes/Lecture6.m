%% Lecture 6 Matlab code


%% 6.2 A very simple 1 joint planar robot

clc
clear
close all

% We simulate a one rotation joint (arm)

a1=1; % Length of the arm
q1=0.2; % Angle of the arm

trchain2('R(q1) Tx(a1)',q1)

syms q1 a1
trchain2('R(q1) Tx(a1)',q1)

mdl_planar1 %Creates a model of planar 1 joint
p1.teach %Open the graphical simulation

p1.fkine(0.3) % Returns the homtrans for a 0.3rad angle

%% 6.3 A 2 joint planar robot

clc
clear
close all

a1=1;
a2=1;
q1=0.2;
q2=0.3;
trchain2('R(q1) Tx(a1) R(q2) Tx(a2)',[q2 q1])

syms a1 q1 a2 q2
trchain2('R(q1) Tx(a1) R(q2) Tx(a2)',[q2 q1])

mdl_planar2
p2.teach

pause;
p2.plot([0 pi/2]) % Now we show two config with the same position (but not same orientation)
pause;

p2.fkine([0.3 -0.3]) % Exercise
p2.plot([pi/2 -pi/2])

%% 6.4 A 3 joint planar robot

clc
clear
close all

a1=1;
a2=1;
a3=1;
q1=0.2;
q2=0.3;
q3=-0.3;

trchain2('R(q1) Tx(a1) R(q2) Tx(a2) R(q3) Tx(a3)',[q1 q2 q3])

syms a1 q1 a2 q2 a3 q3
trchain2('R(q1) Tx(a1) R(q2) Tx(a2) R(q3) Tx(a3)',[q1 q2 q3])

mdl_planar3
p3.teach

pause;
p3.plot([0 pi/2 pi/2]) % Now we show two config with the same position (but not same orientation)
pause;

p3.fkine([0.3 -0.2 0.3]) % Exercise

%% 6.5 A 3D robot

clc
clear
close all

syms a1 a2 a3 a4 q1 q2 q3 q4

trchain('Rz(q1) Tz(a1) Ry(q2) Tz(a2) Ry(q3) Tz(a3) Ry(q4) Tz(a4)',[q1 q2 q3 q4])

x=ans(1,4)
y=ans(2,4)

mdl_phantomx
px.teach

px.fkine([0.1, 0.2, 0.3, 0.4])

tr2rpy(ans)

%% 6.6 A general 3D robot

% Most robots are 6 revolutionary joints in serie

clc
clear
close all

mdl_puma560
p560.teach

%% 6.7 Task and configuration space

% Configuration space is the number of parameters (q1,q2,...)
% Task space is the number of param to describe the end effector

%% 6.8 Denavit-Hartenberg Parametrisation

clc
clear
close all

%Lets create a denavit hartenberg matrix
dh = [
    0 0 1 0 %first link (theta,d,a,alpha)
    0 0 1 0 %second link (theta,d,a,alpha)
    ]

r = SerialLink(dh) %Creates the robot corresponding to dh

r.plot([0.5 0.3]) % plot the robot with values in q1 q2
pause;
close

r.teach
pause;
close

r.fkine([0.2 0.3])

mdl_puma560
p560

p560.plot(qz)
pause;
p560.plot(qr)
pause;
close

p560.fkine([0.1 0.2 0.3 0 0 0])

%% 6.9 Base and tool transform

clc
clear
close all

mdl_puma560
p560
p560.fkine([0.1 0.2 0.3 0 0 0])
%We see that the base and tool frames are null

p560.base = transl(10,15,2) * trotx(pi) % We add a base transform
p560.fkine([0.1 0.2 0.3 0 0 0]) % We see that the forxard kinematics has changes

p560.tool = transl(0,0,0.2)% We add a tool transform
p560.fkine([0.1 0.2 0.3 0 0 0])
pause;
clear

%Exercise
mdl_puma560
p560.plot(qn)
p560.fkine(qn)
p560.tool=transl(0,0,0.15);
p560.fkine(qn)

p560.base = transl(2,3,0.8);
p560.fkine(qn)

%% Summary

% * Forward kinematics : get the pose of the end effector knowing the pose of
%   the joints
% * Configuration space : depends on joints configuration vectors
% * Task space : space of all possible position & orientation of the
%   end-effector
% * Config space >= Task space (If more we can control the shape of the
%   arm)
% * Denavit Hartenberg : to use only 4 parameters, need to place the link
%   frames in a certain way, can be resumed in a table
% * Base transform : Gives us the pose of the base of the robot in the
%   world frame
% * Tool transform : Gives us the pose of the tool compared to the end of
%   the chain
