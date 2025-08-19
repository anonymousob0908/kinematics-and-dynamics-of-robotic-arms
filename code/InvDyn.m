clear; clc;
addpath 'D:\MATLAB\ModernRobotics-master\packages\MATLAB\mr'
t=0:2*pi/3600:2*pi;
x=16*sin(t).^3/30;
y=(13*cos(t)-5*cos(2*t)-2*cos(3*t)-cos(4*t))/30;
z=0.5*ones(1,length(t));


% 锁死任意角度下的逆动力学
theta3=0;
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

Tsd=zeros(4,4,length(t));
% Spatial Twist
Slist = [[omega1; v1], ...
        [omega2; v2],...
        [omega4; v4],...
        [omega5; v5],...
        [omega6; v6],...
        [omega7; v7]];   %The joint screw axes in the space frame
for i=1:length(t)
    Tsd(:,:,i) = [1,0,0,x(i);
                  0,1,0,y(i);
                  0,0,1,z(i);
                  0,0,0,1];
end
thetalist0 = [pi/2; pi/2; pi/2; pi/2; pi/2; pi/2];
eomg = 1e-8;
ev = 1e-8;
thetalist=zeros(6,length(t));
success=zeros(1,length(t));
for i=1:length(t)
    [thetalist(:,i), success(i)] = IKinSpace(Slist, M, Tsd(:,:,i), thetalist0, eomg, ev);
    thetalist0 = thetalist(:,i);
end

%正运动学验算
M=[1 0 0 0;0 1 0 0;0 0 1 0.3393+0.3977+0.3925+0.2145;0 0 0 1];  % Home Configuration of End-Effector 
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
% Spatial Twist
Slist = [[omega1; v1], ...
        [omega2; v2],...
        [omega3; v3],...
        [omega4; v4],...
        [omega5; v5],...
        [omega6; v6],...
        [omega7; v7]];   %The joint screw axes in the space frame
Tbs=zeros(4,4,length(t));
for i=1:length(t)
    thetalist1 = thetalist(:,i);
    thetalist1 =[thetalist1(1:2); 0; thetalist1(3:end)];
    Tbs(:,:,i)=FKinSpace(M,Slist,thetalist1);
end

% %可视化
% figure(1)
% % 定义原始坐标系（单位矩阵）
% T_origin = eye(4); % 初始坐标系
% trajectory = Tbs(1:3,4,:);  % 更新轨迹
% trajectory = squeeze(trajectory)';
% for i=1:length(t)
%     % 定义变换后的齐次矩阵
%     T_transform = Tbs(:,:,i); % 变换后的坐标系
%     clf;
%     if size(trajectory, 1) > 1
%         plot3(trajectory(:,1), trajectory(:,2), trajectory(:,3), 'r--', 'LineWidth', 2);  % 轨迹线
%     end
%     hold on;
%     % 绘制原始坐标系
%     drawFrame(T_origin, 'g', 'O');
%     hold on;
% 
%     % 绘制变换后的坐标系
%     drawFrame(T_transform, 'b', 'T');
% 
%     % 设置3D视图
%     axis equal;
%     grid on;
%     xlabel('X');
%     ylabel('Y');
%     zlabel('Z');
%     xlim([-1,1]);
%     ylim([-1,1]);
%     title('Transformation Visualization');
%     view(3);  % 3D视角
%     drawnow;
%     %pause(0.05);
% end

%%
% 逆动力学
thetamat=thetalist';
N=length(t);
dthetamat = zeros(N, 6);
ddthetamat = zeros(N, 6);
Tf=1;
dt = Tf / (N - 1);

for i = 2: N - 1
    dthetamat(i, :) = (thetamat(i + 1, :) - thetamat(i - 1, :)) / dt / 2;
end
for i = 2: N-1
    ddthetamat(i, :) = (dthetamat(i + 1, :) - dthetamat(i - 1, :)) / dt / 2;
end

%Initialise robot descripstion (Example with 3 links)
g = [0; 0; -9.8];
Ftipmat = zeros(N, 6); 
%关节齐次坐标变换矩阵
T01 = [[1, 0, 0, 0]; [0, 1, 0, 0]; [0, 0, 1, 0.3393]; [0, 0, 0, 1]];
T02 = [[1, 0, 0, 0]; [0, 0, 1, 0]; [0, -1 ,0, 0.3393]; [0, 0, 0, 1]];
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
%%
% 锁定了joint3,这里假设是机械锁定方式,link2与link3成为了一刚体，刚体质心位置为：
m2=m2+m3;
l2s=(m2*l2s+m3*l3s)/(m2+m3);
%将惯量转换到质心系，质心系在{S}下的坐标,link[i]的质心系的姿态设定为与joint[i]的姿态一致，平行轴定理：
I1=I1+m1*(l1(1:3,1)'*l1(1:3,1)*eye(3)-l1(1:3,1)*l1(1:3,1)');
I2=I2+m2*(l2(1:3,1)'*l2(1:3,1)*eye(3)-l2(1:3,1)*l2(1:3,1)');
I3=I3+m3*(l3(1:3,1)'*l3(1:3,1)*eye(3)-l3(1:3,1)*l3(1:3,1)');
I4=I4+m4*(l4(1:3,1)'*l4(1:3,1)*eye(3)-l4(1:3,1)*l4(1:3,1)');
I5=I5+m5*(l5(1:3,1)'*l5(1:3,1)*eye(3)-l5(1:3,1)*l5(1:3,1)');
I6=I6+m6*(l6(1:3,1)'*l6(1:3,1)*eye(3)-l6(1:3,1)*l6(1:3,1)');
I7=I7+m7*(l7(1:3,1)'*l7(1:3,1)*eye(3)-l7(1:3,1)*l7(1:3,1)');
%%
% 锁定joint3，link2与link3的等效惯量发生变化，即I2需等效为I2与I3的共同作用
%等效惯量计算
%3等效后的齐次坐标变换矩阵
T02d=[T02(1:3,1:3),l2s(1:3,1);0,0,0,1];
T2d3=TransInv(T02d)*T03;
R2d3=T2d3(1:3,1:3);
l2d3=T2d3(1:3,4);
%I3等效(旋转+平移)
I3=R2d3*I3*R2d3'+m3*(l2d3'*l2d3*eye(3)-l2d3*l2d3');
%2等效后的齐次坐标变换矩阵
T2d2=TransInv(T02d)*T02;
l2d2=T2d3(1:3,4);
%I2等效(只平移)
I2=I2+m2*(l2d2'*l2d2*eye(3)-l2d2*l2d2');
%等效惯量
I2=I2+I3;
%%
%质心系初始位置
M01=[T01(1:3,1:3),l1s(1:3,1);0,0,0,1];
M02=[T02(1:3,1:3),l2s(1:3,1);0,0,0,1];
M04=[T04(1:3,1:3),l4s(1:3,1);0,0,0,1];
M05=[T05(1:3,1:3),l5s(1:3,1);0,0,0,1];
M06=[T06(1:3,1:3),l6s(1:3,1);0,0,0,1];
M07=[T07(1:3,1:3),l7s(1:3,1);0,0,0,1];
M08=M07;%无末端连杆
%link[i]的质心系在{i-1}下的坐标
M12=TransInv(M01)*M02;
M24=TransInv(M02)*M04;
M45=TransInv(M04)*M05;
M56=TransInv(M05)*M06;
M67=TransInv(M06)*M07;
M78=eye(4);

%广义惯量矩阵
G1 = [I1,zeros(3);zeros(3),m1*eye(3)];
G2 = [I2,zeros(3);zeros(3),m2*eye(3)];
G4 = [I4,zeros(3);zeros(3),m4*eye(3)];
G5 = [I5,zeros(3);zeros(3),m5*eye(3)];
G6 = [I6,zeros(3);zeros(3),m6*eye(3)];
G7 = [I7,zeros(3);zeros(3),m7*eye(3)];

Glist = cat(3, G1, G2, G4, G5, G6, G7);
Mlist = cat(3, M01, M12, M24, M45, M56, M67, M78); 
Slist = [[omega1; v1], ...
        [omega2; v2],...
        [omega4; v4],...
        [omega5; v5],...
        [omega6; v6],...
        [omega7; v7]];   %The joint screw axes in the space frame
taumat = InverseDynamicsTrajectory(thetamat, dthetamat, ddthetamat, ...
                                 g, Ftipmat, Mlist, Glist, Slist);

%%
figure(2)
%Output using matplotlib to plot the joint forces/torques
time=0: dt: Tf;
plot(time, taumat(:, 1))
hold on
plot(time, taumat(:, 2))
plot(time, taumat(:, 3))
plot(time, taumat(:, 4))
hold on
plot(time, taumat(:, 5))
plot(time, taumat(:, 6))
title('Plot for Torque Trajectories')
xlabel('Time')
ylabel('Torque')
legend('Tau1', 'Tau2', 'Tau4', 'Tau5', 'Tau6', 'Tau7')

%%
% 自定义函数
function drawFrame(T, color, label)
    R = T(1:3, 1:3);  % 旋转矩阵
    p = T(1:3, 4);    % 平移向量
    
    % 绘制X轴
    axes_end = p + R(:,1);  % X轴终点
    quiver3(p(1), p(2), p(3), R(1,1), R(2,1), R(3,1), 'Color', color, 'LineWidth', 2);
    text(axes_end(1), axes_end(2), axes_end(3), ['X_' label], 'Color', color);
    
    % 绘制Y轴
    axes_end = p + R(:,2);  % Y轴终点
    quiver3(p(1), p(2), p(3), R(1,2), R(2,2), R(3,2), 'Color', color, 'LineWidth', 2);
    text(axes_end(1), axes_end(2), axes_end(3), ['Y_' label], 'Color', color);
    
    % 绘制Z轴
    axes_end = p + R(:,3);  % Z轴终点
    quiver3(p(1), p(2), p(3), R(1,3), R(2,3), R(3,3), 'Color', color, 'LineWidth', 2);
    text(axes_end(1), axes_end(2), axes_end(3), ['Z_' label], 'Color', color);
end