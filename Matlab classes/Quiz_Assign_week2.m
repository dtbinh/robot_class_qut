%% Quiz and Assignement week 2

%% Quiz Week 2

clc
clear all
close all

% Question 5

[s,sd,sdd] = lspb(1.2,2.4,80);% We want the move to last 8s => 80steps
max(sd) % Max speed is 0.0228

lspb(1.2,2.4,80,0.02) % With a velocity of 0.02 the acceleration lasts 2s (20steps)
%pause;
lspb(3.7,2.7,80,0.0166) % Now for the second joint (y axis) we need a speed of 16.6mm/s to have an initial acc of 2s
%pause;

% Question 6

clc 
clear all
close all

% Linear interpolation for RPY angles

Li1 = ((1-0.7)*[10 -5 15]'+0.7*[47 16 -33]')'

% ZYZ euler

e1 = tr2eul(rpy2tr(10,-5,15,'deg'),'deg');
e2 = tr2eul(rpy2tr(47,16,-33,'deg'),'deg');

Li2eul = (1-0.7)*e1'+0.7*e2';

Li2 = tr2rpy(eul2tr(Li2eul','deg'),'deg')

%Quaternion

q1 = Quaternion(rpy2r(10,-5,15,'deg')); 
q2 = Quaternion(rpy2r(47,16,-33,'deg')); 

init = q1.interp(q2,0);%Second argument is the position along the trajectory, 0 is first point
final = q1.interp(q2,1);% 1 is last point
int = q1.interp(q2,0.7);% 70% 

Li3 = tr2rpy(int.R,'deg')

%% Matlab Assignment week 2

clc
clear all
close all

% DO NOT CHANGE THE FOLLOWING LINES
V = [
-1  1  1 -1  -1  1  1 -1  
-1 -1  1  1  -1 -1  1  1
-1 -1 -1 -1   1  1  1  1 ];

edges = [
     1     2     3     4     5     6     7     8     1     2     3     4
     2     3     4     1     6     7     8     5     5     6     7     8 ];

P = [5, 6, 4];
C = transl(P) * oa2tr([0 1 0], -P);

% Modify the following lines to generate the plot
figure
axis ([-2 6 -2 7 -2 6])
axis square
xlabel('X')
ylabel('Y')
zlabel('Z')
hold on
grid on
rotate3d on
% Plot the original square and the camera frame
for i=1:12
    plot3([V(1,edges(1,i)) V(1,edges(2,i))],[V(2,edges(1,i)) V(2,edges(2,i))],[V(3,edges(1,i)) V(3,edges(2,i))])
end
trplot(C)

pause;

figure
axis square
xlabel('X')
ylabel('Y')
zlabel('Z')
hold on
grid on
rotate3d on

V_C = zeros(4,8);
% Apply the tranformation to the cube coordinates
for i=1:8
V_C(:,i)=inv(C)*[V(:,i);1];
end

V_C=V_C(1:3,:);

% Plot the cube camera coordinates
for i=1:12
    plot3([V_C(1,edges(1,i)) V_C(1,edges(2,i))],[V_C(2,edges(1,i)) V_C(2,edges(2,i))],[V_C(3,edges(1,i)) V_C(3,edges(2,i))])
end

%% Matlab assignement week 2 part2

clc
clear all
close all

% DO NOT CHANGE THE FOLLOWING LINES
P0 = [5, 6, 4]/2;
C0 = transl(P0) * oa2tr([0 1 0], -P0);
Pi = [5, 6, 4];
Ci = transl(Pi) * oa2tr([0 1 0], -Pi);
Pf = [-6, 6, 8];
Cf = transl(Pf) * oa2tr([0 1 0], -Pf);

figure
hold on
grid on
axis square
rotate3d on
xlabel('X')
ylabel('Y')
zlabel('Z')

% Plot the three frames
trplot(C0,'frame','C0','color','k');
trplot(Ci,'frame','C0','color','b');
trplot(Cf,'frame','C0','color','g');


figure
hold on
grid on
axis square
xlabel('X')
ylabel('Y')
zlabel('Z')

% Modify the following lines to compute your answer


%Translation part

start = P0;
via = [Pi;Pf];

xt = mstraj(via,[],[6 8],start,0.2,1.5); 

plot(xt)

TR = transl(xt);%% Contain the translation param vs time

figure
plot3(xt(:,1),xt(:,2),xt(:,3))
xlabel('X')
ylabel('Y')
zlabel('Z')
grid on
axis square
rotate3d on

%tranimate(TR)

%Rotation part

start = tr2rpy(C0);

via = [tr2rpy(Ci); tr2rpy(Cf)];

xr = mstraj(via,[],[6 8],start,0.2,1.5);

figure;

xrpy=rpy2tr(xr); %% Contain the rotation param vs time

T=zeros(4,4,70);

for i=1:70
    T(:,:,i)=TR(:,:,i)*xrpy(:,:,i);
end

%% correction part 2

    V0 = [ transl(C0)'  tr2rpy(C0) ];
    Vf = [ transl(Cf)'  tr2rpy(Cf) ];
    Vi = [ transl(Ci)'   tr2rpy(Ci) ];

    
	TRAJ = mstraj([Vi; Vf], [], [6 8], V0, 1/5, 1.5)


    P50 = TRAJ(50,:);
    
    
	C50 = transl( P50(1:3) ) * rpy2tr( P50(4:6) );
    
    