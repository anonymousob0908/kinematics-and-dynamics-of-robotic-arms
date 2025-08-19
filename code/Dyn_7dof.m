addpath D:\MATLAB\ModernRobotics-master\packages\MATLAB\mr
t=0:2*pi/3600:2*pi;
N=length(t);
Tf=1;
dt = Tf / (N - 1);
intRes=10;
thetalist=[-0.240774643118505;1.64709986424071;1.87386688633768;3.58838087271826;1.87742907734026;1.63066270814506;-0.185102255761343];
dthetalist=[0;0;0;0;0;0;0];
%Initialise robot descripstion (Example with 3 links)
g = [0; 0; -9.8];
Ftipmat = zeros(N, 6); 
%关节齐次坐标变换矩阵
T01 = [[1, 0, 0, 0]; [0, 1, 0, 0]; [0, 0, 1, 0.3393]; [0, 0, 0, 1]];
T02 = [[1, 0, 0, 0]; [0, 0, 1, 0]; [0, -1 ,0, 0.3393]; [0, 0, 0, 1]];
T03 = [[1, 0, 0, 0]; [0, 1, 0, 0]; [0, 0, 1, 0.3393+0.3977]; [0, 0, 0, 1]];
T04 = [[1, 0, 0, 0]; [0, 0, 1, 0]; [0, -1, 0, 0.3393+0.3977]; [0, 0, 0, 1]];
T05 = [[1, 0, 0, 0]; [0, 1, 0, 0]; [0, 0, 1, 0.3393+0.3977+0.3925]; [0, 0, 0, 1]];
T06 = [[1, 0, 0, 0]; [0, 0, 1, 0]; [0, -1, 0, 0.3393+0.3977+0.3925]; [0, 0, 0, 1]];
T07 = [[1, 0, 0, 0]; [0, 1, 0, 0]; [0, 0, 1, 0.3393+0.3977+0.3925+0.2145]; [0, 0, 0, 1]];
M = T07;
omega1=[0;0;1];
omega2=[0;-1;0];
omega3=[0;0;1];
omega4=[0;-1;0];
omega5=[0;0;1];
omega6=[0;-1;0];
omega7=[0;0;1];
p1=[0;0;0.3393];
p2=[0;0;0.3393];
p3=[0;0;0.3393+0.3977];
p4=[0;0;0.3393+0.3977];
p5=[0;0;0.3393+0.3977+0.3925];
p6=[0;0;0.3393+0.3977+0.3925];
p7=[0;0;0.3393+0.3977+0.3925+0.2145];
v1=-cross(omega1,p1);
v2=-cross(omega2,p2);
v3=-cross(omega3,p3);
v4=-cross(omega4,p4);
v5=-cross(omega5,p5);
v6=-cross(omega6,p6);
v7=-cross(omega7,p7);
%质心系
%质心在连体基下的坐标
l1=[0;0.007;-0.063;1];
l2=[0;0.115;0.025;1];
l3=[0;0.008;-0.044;1];
l4=[0;0.13;0.016;1];
l5=[0;0.007;-0.061;1];
l6=[0;0.078;0.016;1];
l7=[0;0;-0.029;1];
%质心在{S}系下的坐标
l1s=T01*l1;
l2s=T02*l2;
l3s=T03*l3;
l4s=T04*l4;
l5s=T05*l5;
l6s=T06*l6;
l7s=T07*l7;
%惯量与质量
I1 = diag([0.056, 0.055, 0.01]);
I2 = diag([0.099, 0.01, 0.095]);
I3 = diag([0.02, 0.019, 0.004]);
I4 = diag([0.008, 0.005, 0.078]);
I5 = diag([0.023, 0.022, 0.03]);
I6 = diag([0.024, 0.003, 0.024]);
I7 = diag([0.001, 0.001, 0.001]);
m1=5.171;
m2=3.958;
m3=3.078;
m4=3.093;
m5=2.549;
m6=2.063;
m7=0.907;
%将惯量转换到质心系，质心系在{S}下的坐标,link[i]的质心系的姿态设定为与joint[i]的姿态一致，平行轴定理：
I1=I1+m1*(l1(1:3,1)'*l1(1:3,1)*eye(3)-l1(1:3,1)*l1(1:3,1)');
I2=I2+m2*(l2(1:3,1)'*l2(1:3,1)*eye(3)-l2(1:3,1)*l2(1:3,1)');
I3=I3+m3*(l3(1:3,1)'*l3(1:3,1)*eye(3)-l3(1:3,1)*l3(1:3,1)');
I4=I4+m4*(l4(1:3,1)'*l4(1:3,1)*eye(3)-l4(1:3,1)*l4(1:3,1)');
I5=I5+m5*(l5(1:3,1)'*l5(1:3,1)*eye(3)-l5(1:3,1)*l5(1:3,1)');
I6=I6+m6*(l6(1:3,1)'*l6(1:3,1)*eye(3)-l6(1:3,1)*l6(1:3,1)');
I7=I7+m7*(l7(1:3,1)'*l7(1:3,1)*eye(3)-l7(1:3,1)*l7(1:3,1)');

%%
%质心系初始位置
M01=[T01(1:3,1:3),l1s(1:3,1);0,0,0,1];
M02=[T02(1:3,1:3),l2s(1:3,1);0,0,0,1];
M03=[T03(1:3,1:3),l3s(1:3,1);0,0,0,1];
M04=[T04(1:3,1:3),l4s(1:3,1);0,0,0,1];
M05=[T05(1:3,1:3),l5s(1:3,1);0,0,0,1];
M06=[T06(1:3,1:3),l6s(1:3,1);0,0,0,1];
M07=[T07(1:3,1:3),l7s(1:3,1);0,0,0,1];
M08=M07;%无末端连杆
%link[i]的质心系在{i-1}下的坐标
M12=TransInv(M01)*M02;
M23=TransInv(M02)*M03;
M34=TransInv(M03)*M04;
M45=TransInv(M04)*M05;
M56=TransInv(M05)*M06;
M67=TransInv(M06)*M07;
M78=eye(4);

%广义惯量矩阵
G1 = [I1,zeros(3);zeros(3),m1*eye(3)];
G2 = [I2,zeros(3);zeros(3),m2*eye(3)];
G3 = [I3,zeros(3);zeros(3),m3*eye(3)];
G4 = [I4,zeros(3);zeros(3),m4*eye(3)];
G5 = [I5,zeros(3);zeros(3),m5*eye(3)];
G6 = [I6,zeros(3);zeros(3),m6*eye(3)];
G7 = [I7,zeros(3);zeros(3),m7*eye(3)];

Glist = cat(3, G1, G2, G3, G4, G5, G6, G7);
Mlist = cat(3, M01, M12, M23, M34, M45, M56, M67, M78); 
Slist = [[omega1; v1], ...
        [omega2; v2],...
        [omega3; v3],...
        [omega4; v4],...
        [omega5; v5],...
        [omega6; v6],...
        [omega7; v7]];   %The joint screw axes in the space frame
load tau_7dof.mat
load('theta_7dof.mat');
thetamat_InvDyn = thetamat;  % 复制到新变量名
clear thetamat;              % 删除原变量
load('dtheta_7dof.mat');
dthetamat_InvDyn = dthetamat;  % 复制到新变量名
clear dthetamat;              % 删除原变量
[thetamat,dthetamat]=ForwardDynamicsTrajectory(thetalist,dthetalist,taumat,g,Ftipmat,Mlist,Glist,Slist,dt,intRes);
%%
delta_theta=thetamat-thetamat_InvDyn;
delta_dtheta=dthetamat-dthetamat_InvDyn;

%%
figure(1)
time=0: dt: Tf;
plot(time, thetamat(:, 1))
hold on
plot(time, thetamat(:, 2))
plot(time, thetamat(:, 3))
plot(time, thetamat(:, 4))
hold on
plot(time, thetamat(:, 5))
plot(time, thetamat(:, 6))
plot(time, thetamat(:, 7))
title('Plot for \theta')
xlabel('Time')
ylabel('\theta')
legend('\theta_1', '\theta_2', '\theta_3', '\theta_4', '\theta_5', '\theta_6', '\theta_7')
figure(2)
plot(time, dthetamat(:, 1))
hold on
plot(time, dthetamat(:, 2))
plot(time, dthetamat(:, 3))
plot(time, dthetamat(:, 4))
hold on
plot(time, dthetamat(:, 5))
plot(time, dthetamat(:, 6))
plot(time, dthetamat(:, 7))
title('Plot for $\dot{\theta}$', 'Interpreter', 'latex')
xlabel('Time')
ylabel('$\dot{\theta}$', 'Interpreter', 'latex')
legend('$\dot{\theta_1}$', '$\dot{\theta_2}$', '$\dot{\theta_4}$', '$\dot{\theta_5}$', '$\dot{\theta_6}$', '$\dot{\theta_7}$','Interpreter', 'latex')
figure(3)
plot(time, delta_theta(:, 1))
hold on
plot(time, delta_theta(:, 2))
plot(time, delta_theta(:, 3))
plot(time, delta_theta(:, 4))
hold on
plot(time, delta_theta(:, 5))
plot(time, delta_theta(:, 6))
title('Plot for \Delta\theta')
xlabel('Time')
ylabel('\Delta\theta')
legend('\Delta\theta_1', '\Delta\theta_2', '\Delta\theta_4', '\Delta\theta_5', '\Delta\theta_6', '\Delta\theta_7')
figure(4)
plot(time, delta_dtheta(:, 1))
hold on
plot(time, delta_dtheta(:, 2))
plot(time, delta_dtheta(:, 3))
plot(time, delta_dtheta(:, 4))
hold on
plot(time, delta_dtheta(:, 5))
plot(time, delta_dtheta(:, 6))
title('Plot for $\Delta\dot{\theta}$', 'Interpreter', 'latex');
xlabel('Time')
ylabel('$\Delta\dot{\theta}$', 'Interpreter', 'latex')
legend('$\Delta\dot{\theta_1}$', '$\Delta\dot{\theta_2}$', '$\Delta\dot{\theta_4}$', '$\Delta\dot{\theta_5}$', '$\Delta\dot{\theta_6}$', '$\Delta\dot{\theta_7}$','Interpreter', 'latex')