% M=[1 0 0 0;0 1 0 0;0 0 1 0.3393+0.3977+0.3925+0.2145;0 0 0 1];  % Home Configuration of End-Effector 
% p1=[0;0;0.3393];
% p2=[0;0;0.3393];
% % p3=[0;0;0.3393+0.3977];
% p4=[0;0;0.3393+0.3977];
% p5=[0;0;0.3393+0.3977+0.3925];
% p6=[0;0;0.3393+0.3977+0.3925];
% p7=[0;0;0.3393+0.3977+0.3925+0.2145];
% 
% omega1=[0;0;1];
% omega2=[0;-1;0];
% % omega3=[0;0;1];
% omega4=[0;-1;0];
% omega5=[0;0;1];
% omega6=[0;-1;0];
% omega7=[0;0;1];
% 
% v1=-cross(omega1,p1);
% v2=-cross(omega2,p2);
% % v3=-cross(omega3,p3);
% v4=-cross(omega4,p4);
% v5=-cross(omega5,p5);
% v6=-cross(omega6,p6);
% v7=-cross(omega7,p7);
% %% Spatial Twist
% addpath 'D:\MATLAB\ModernRobotics-master\packages\MATLAB\mr'
% Slist = [[omega1; v1], ...
%         [omega2; v2],...
%         [omega4; v4],...
%         [omega5; v5],...
%         [omega6; v6],...
%         [omega7; v7]];   %The joint screw axes in the space frame
% Tsd = Tbs1;
% thetalist0 = [pi/6+0.1; pi/6+0.1; pi/6+0.1; pi/6+0.1; pi/6+0.1; pi/6+0.1];
% eomg = 0.01;
% ev = 0.001;
% 
% [thetalist, success] = IKinSpace(Slist, M, Tsd, thetalist0, eomg, ev)

%%
% 锁死任意角度下的逆动力学
theta3=pi/6;
T34=[1,0,0,0;
     0,0,-1,0;
     0,1,0,0;
     0,0,0,1];
T35=[1,0,0,0;
     0,1,0,0;
     0,0,1,0.3925;
     0,0,0,1];
T36=[1,0,0,0;
     0,0,-1,0;
     0,1,0,0.3925;
     0,0,0,1];
T37=[1,0,0,0;
     0,1,0,0;
     0,0,1,0.3925+0.2145;
     0,0,0,1];
T03=[cos(theta3),-sin(theta3),0,0;
     sin(theta3),cos(theta3),0,0;
     0,0,1,0.3393+0.3977;
     0,0,0,1];
T04=T03*T34;
T05=T03*T35;
T06=T03*T36;
T07=T03*T37;
M=T07;
omega1=[0;0;1];
omega2=[0;-1;0];
omega4=T04(1:3,3);
omega5=T05(1:3,3);
omega6=T06(1:3,3);
omega7=T07(1:3,3);

p1=[0;0;0.3393];
p2=[0;0;0.3393];
p4=T04(1:3,4);
p5=T05(1:3,4);
p6=T06(1:3,4);
p7=T07(1:3,4);
v1=-cross(omega1,p1);
v2=-cross(omega2,p2);
v4=-cross(omega4,p4);
v5=-cross(omega5,p5);
v6=-cross(omega6,p6);
v7=-cross(omega7,p7);
%% Spatial Twist
addpath 'D:\MATLAB\ModernRobotics-master\packages\MATLAB\mr'
Slist = [[omega1; v1], ...
        [omega2; v2],...
        [omega4; v4],...
        [omega5; v5],...
        [omega6; v6],...
        [omega7; v7]];   %The joint screw axes in the space frame
Tsd = Tbs1;
thetalist0 = [pi/6+0.1; pi/6+0.1; pi/6+0.1; pi/6+0.1; pi/6+0.1; pi/6+0.1];
eomg = 0.01;
ev = 0.001;
    
[thetalist, success] = IKinSpace(Slist, M, Tsd, thetalist0, eomg, ev)
