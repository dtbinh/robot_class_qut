%%%% Lecture 2 matlab code 

% Don't forget to add the robotic toolbox in matlab path :)

%% 2.6


% Function from robotic toolbox that writes the rotation matrix of the
% corresponding angle
rot2(0)

rot2(0.2)

rot2(30,'deg')

R=ans

c1 = R(:,1)
c2 = R(:,2)

% The dot product of the two columns is 0 (rotation matrices are
% orthogonals)
dot(c1,c2)

% we see that det = 1 for rotation matrices
det(R)

% we can see that inv(R) = R' which is super useful
inv(R)

R'


% plot the R frame
trplot2(R)
axis equal 
grid on


%% 2.7

% Function from robotic toolbox that writes the homogeneous transformation
% matrix for the 2D translation (1,2)
transl2(1,2)

rot2(30,'deg')
% Returns the homogeneous transform matrix for the 2D rotation 
trot2(30, 'deg')

% Multiplies the two transform matrices to get a rotation and a translation
% in the transform matrix
transl2(1,2)*trot2(30,'deg')

% Equivalent in one function 
se2(1,2,30,'deg')

% Frames ploting
axis([0 5 0 5])
axis square
hold on0 5])
axis square
hold on

T1 = se2(1,2,30,'deg')
trplot2(T1,'frame','1','color','b')

T2 = se2(2,1,0)
trplot2(T2,'frame','2','color','r')

T3=T1*T2
trplot2(T3,'frame','3','color','g') % We can see first T1 and then T2

T4=T2*T1 % We try the opposite
trplot2(T4,'frame','4','color','c')

P = [3,2]' % Lets create a point at (3,2) in the world coordinates
plot_point(P,'*')
 
P1 = inv(T1)* [P;1] % Returns the coord of P in frame 1 ! 

%% Assignement part 1
close all
clear all
clc

% DO NOT CHANGE THE FOLLOWING LINES
PA_0 = [6.5; 4.3];  % point A with respect to {0}
PB_2 = [1.2; -2.7];  % point B with respect to {2}

% Modify the following lines to return the correct values
R1 = rot2(-50,'deg');
T1 = transl2(3,4);
T2 = se2(5,6,-30,'deg');
PA_2 = inv(T2) * [PA_0;1];
PB_0 = T2 * [PB_2;1];
PB_1 = inv(T1) * PB_0;
D_AB = sqrt((PA_0(1)-PB_0(1))^2 +(PA_0(2)-PB_0(2))^2);

% Better solution with function homtrans

PA_2 = homtrans(inv(T2), PA_0)
PB_0 = homtrans(T2,PB_2);
PB_1 = homtrans(inv(T1),PB_0)

D_AB = norm(PB_0 - PA_0)

axis ([0 10 0 10])
axis square
hold on
grid on

trplot2(T1,'frame','1','color','b')
trplot2(T2,'frame','2','color','r')
plot_point(PA_0,'*')
plot_point(PB_0,'*')

%% Assignement part 2

close all
clear all 
clc

% DO NOT CHANGE THE FOLLOWING LINES
V = [
    -1 1 1 -1
    -1 -1 1 1];
theta = 56;

% Modify the following lines to generate the plot

axis([-1.5 1.5 -1.5 1.5])
axis square
hold on
grid on

% Plot the original square V
%plot(V(:,2),V(:,1),'b')
%plot(V(:,3),V(:,2),'b')
%plot(V(:,4),V(:,3),'b')
%plot(V(:,1),V(:,4),'b')


% We create a frame rotated by theta
T_B = trot2(theta,'deg');

% We calculate the points coordinates in V_World frame knowing the
% coordinates in the T_B frame
V_W(:,1) = homtrans(T_B,V(:,1));
V_W(:,2) = homtrans(T_B,V(:,2));
V_W(:,3) = homtrans(T_B,V(:,3));
V_W(:,4) = homtrans(T_B,V(:,4));

% We plot everything
plot(V_W(:,2),V_W(:,1),'r')
plot(V_W(:,3),V_W(:,2),'r')
plot(V_W(:,4),V_W(:,3),'r')
plot(V_W(:,1),V_W(:,4),'r')
