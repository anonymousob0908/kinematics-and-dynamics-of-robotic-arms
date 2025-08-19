M=[1 0 0 0;0 1 0 0;0 0 1 0.3393+0.3977+0.3925+0.2145;0 0 0 1]  % Home Configuration of End-Effector 
p1=[0;0;0.3393];
p2=[0;0;0.3393];
p3=[0;0;0.3393+0.3977];
p4=[0;0;0.3393+0.3977];
p5=[0;0;0.3393+0.3977+0.3925];
p6=[0;0;0.3393+0.3977+0.3925];
p7=[0;0;0.3393+0.3977+0.3925+0.2145];

omega1=[0;0;1];
omega2=[0;-1;0];
omega3=[0;0;1];
omega4=[0;-1;0];
omega5=[0;0;1];
omega6=[0;-1;0];
omega7=[0;0;1];

v1=-cross(omega1,p1);
v2=-cross(omega2,p2);
v3=-cross(omega3,p3);
v4=-cross(omega4,p4);
v5=-cross(omega5,p5);
v6=-cross(omega6,p6);
v7=-cross(omega7,p7);

addpath 'D:\MATLAB\ModernRobotics-master\packages\MATLAB\mr'
%% Spatial Twist
Slist = [[omega1; v1], ...
        [omega2; v2],...
        [omega3; v3],...
        [omega4; v4],...
        [omega5; v5],...
        [omega6; v6],...
        [omega7; v7]];   %The joint screw axes in the space frame
thetalist = [pi/6; pi/6; pi/6; pi/6; pi/6; pi/6; pi/6];

Tbs1=FKinSpace(M,Slist,thetalist)