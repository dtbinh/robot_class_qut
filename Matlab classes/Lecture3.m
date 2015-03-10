%% Lecture 3 matlab code

%% 3.6 Describing rotation
clc
clear all 
close all

% Rotation matrix around y axis of 30°
RY30 = roty(30,'deg');

trplot(RY30,'frame','A');
hold on

RX60 = rotx(60,'deg');
trplot(RX60,'frame','B','color','r');

% We compare transpose and inverse => equal
inv(RY30);

RY30';

% Multiply by its transpose(inverse) =W give identity
RY30'*RY30

%Compose the two rotations
RC = RY30*RX60;
trplot(RC,'frame','C','color','g');


%% 3.7 Rotations are non communtative !
clc
clear all
close all

% We show that the order in which you rotate is really important
RA = rotx(0);
RB = rotz(pi/2)*rotx(pi/2);
RC = rotx(pi/2)*rotz(pi/2);


trplot(RA,'color','k');
hold on
trplot(RB,'color','b');
trplot(RC,'color','g');

%% 3.8 Angle sequences

clc
clear all
close all

%Convert euler ZYZ convention into roation matrix
R = eul2r([30,40,50],'deg')
%Convert rotation matrix into ZYZ euler convention
E = tr2eul(R,'deg')
%Convert rotation matric into Roll Pitch Yaw angle convetion
RPY = tr2rpy(R,'deg')
%Convert Roll Pitch Yaw angles into rotation matrix
R2 = rpy2r([30,40,50],'deg')

%% 3.9 Singularity

clc
clear all
close all

% When you design something you need to define pitch angle as 0° on nominal
% position so that it stays away from 90°

% If pitch = 90° Roll = 0 and yaw = yaw + roll => not good !
% example :
rpy2r(0.3,pi/2,0.5)
tr2rpy(ans)
% New rpy are 0 pi/2 and 0.3+0.5=0.8

%% 3.10 2-Vector representation

clc
clear all
close all

oa2r([-1.8833,  6.1427, 1.0000], [1.0000,   -0.4925,  4.9085])

%% 3.11 Angle-axis representation

clc
clear all
close all

% Any two indep coordinates frames can be related by a single rotation
% Theta about some axis v

% We create a R with random values
R = eul2r(0.1,0.2,0.3)
trplot(R)
% We see that eigen values are 2 complex related to theta and one equal to
% one which corresponds to the vector v
eig(R)

% Here v represents eigen vectors and e represents eigen values
% The eigen vector which correpsonding to the eigen value of 1 (third vector
% in this case) represents the rotation axis v that we need to turn around of
% theta
[v,e] = eig(R)

% To get theta we use the other eigen values :
tr2angvec(R)
[th,v] = tr2angvec(R)

% To do the inverse we use 
angvec2tr(th,v)

%% 3.12 Quaternions 

clc
clear all
close all

% Exercise 1

R = rpy2r(20,-10,30,'deg');
Quaternion(R)

%Exercise 2

% We creates two rotation matrices
R1 = rotx(30,'deg')
R2 = roty(60,'deg')

% We create a Quaternion corresponding to R1
q1 = Quaternion(R1)
q1.R % We check that it corresponds to R1

% We create a Quaternion corresponding to R2
q2 = Quaternion(R2)
q2.plot()

q=q1*q2 % We can multiply quaternions
q.R
R1*R2

% We compare Rot matric of inv of q1 and inv of rot matrix
inv(q1);
ans.R
inv(R1)

q1*inv(q1) % Multiply q1 by its inverse make zero
ans.R

%% 3.13 4x4 Transformation matrix

clc
clear all
close all

% To contain a rotation and a translation in 3D we need a 4x4 Trans matrix

R = transl(1,2,3)*oa2tr([1,0,0], [0,0,-1]);% Important to make translation before rotation
trplot(R)

%% Recap for lecture

% Right hand frames
% Rotation in 3D = 3x3 matrix, colums = coordinates of axis of frame B 
% Three rotation matrices rotx, roty, rotz
% Euler angles = often ZYZ
% Cardan angles = often Raw Pitch Yaw angles (XYZ)
% Axis-angle form
% Quaternions : vector + rotation
% Transformation matrices : useful because we can multiply them
% Careful with RPY and ZYZ to gamble lock

% In robotics we use Transformation matrices and quaternions