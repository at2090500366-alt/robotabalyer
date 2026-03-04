% 创建PUMA560机器人模型
    a1=0;
    a2=0.506628;
    a3=0.020;
    a4=0;
    a5=0;
    a6=0;
    d1=0.77421; 
    d2=0.101592;
    d3=-0.0381;
    d4=0.267969;
    d5= 0;
    d6=0.05842; 
    alpha1 = deg2rad(-90);
    alpha2 = deg2rad(180);
    alpha3 = deg2rad(-90);
    alpha4 = deg2rad(90);
    alpha5 = deg2rad(-90);
    alpha6 = deg2rad(0);


robot = rigidBodyTree('DataFormat','row','MaxNumBodies',6);

% 连杆1
body1 = rigidBody('link1');
joint1 = rigidBodyJoint('joint1','revolute');
setFixedTransform(joint1, [a1, alpha1, d1, 0], 'dh');
body1.Joint = joint1;
addBody(robot, body1, 'base');

% 连杆2
body2 = rigidBody('link2');
joint2 = rigidBodyJoint('joint2','revolute');
setFixedTransform(joint2, [a2, alpha2, d2, 0], 'dh');
body2.Joint = joint2;
addBody(robot, body2, 'link1');

% 连杆3
body3 = rigidBody('link3');
joint3 = rigidBodyJoint('joint3','revolute');
setFixedTransform(joint3, [a3, alpha3, d3, 0], 'dh');
body3.Joint = joint3;
addBody(robot, body3, 'link2');

% 连杆4
body4 = rigidBody('link4');
joint4 = rigidBodyJoint('joint4','revolute');
setFixedTransform(joint4, [a4, alpha4, d4, 0], 'dh');
body4.Joint = joint4;
addBody(robot, body4, 'link3');

% 连杆5
body5 = rigidBody('link5');
joint5 = rigidBodyJoint('joint5','revolute');
setFixedTransform(joint5, [a5, alpha5, d5, 0], 'dh');
body5.Joint = joint5;
addBody(robot, body5, 'link4');

% 连杆6
body6 = rigidBody('link6');
joint6 = rigidBodyJoint('joint6','revolute');
setFixedTransform(joint6, [a6, alpha6, d6, 0], 'dh');
body6.Joint = joint6;
addBody(robot, body6, 'link5');

%计算并可视化PUMA560机械臂的工作空间

% 关节角范围（可根据实际调整）
q1 = linspace(-pi, pi, 15);
q2 = linspace(-pi/2, pi/2, 10);
q3 = linspace(-pi/2, pi/2, 10);
q4 = linspace(-pi, pi, 6);
q5 = linspace(-pi/2, pi/2, 6);
q6 = linspace(-pi, pi, 6);

[X, Y, Z] = deal([]);
for i1 = 1:length(q1)
    for i2 = 1:length(q2)
        for i3 = 1:length(q3)
            for i4 = 1:length(q4)
                for i5 = 1:length(q5)
                    for i6 = 1:length(q6)
                        config = [q1(i1), q2(i2), q3(i3), q4(i4), q5(i5), q6(i6)];
                        T = getTransform(robot, config, robot.BodyNames{end});
                        X(end+1) = T(1,4);
                        Y(end+1) = T(2,4);
                        Z(end+1) = T(3,4);
                    end
                end
            end
        end
    end
end

figure;
scatter3(X, Y, Z, 5, Z, 'filled');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('PUMA560机械臂工作空间');
axis equal;
grid on;


A=[
0 0 0 1 1 1 0 0 0 0 0 0 1 1 1 1 1 1 1 1 0 0 0 1 1 1 1 1 1 1 1 0 0 0 1 1 1 1 1 1 1 1 1 0 1 1 1 1 1 1 1 1 1 1 0 1 1 0 0 0 0 0 1 1 0
0 0 0 0 0 1 0 0 0 0 0 1 0 0 0 0 0 0 0 0 1 0 1 0 0 0 0 0 0 0 0 1 0 0 1 0 0 0 0 0 0 0 1 0 1 0 0 0 0 0 0 0 0 1 0 1 0 1 0 0 0 1 0 1 0
0 1 1 1 0 1 0 0 0 0 0 1 0 1 1 1 1 1 1 0 1 0 1 0 1 1 1 1 1 1 0 1 0 0 1 0 1 1 1 1 1 1 1 0 1 1 1 1 1 1 0 0 1 0 0 1 0 0 1 0 1 0 0 1 0
0 0 0 1 0 1 0 0 0 0 0 1 0 1 0 0 0 0 1 0 1 0 1 0 1 0 0 0 0 1 0 1 0 0 1 0 1 0 0 0 0 0 0 0 0 0 0 0 0 1 0 1 0 0 0 0 1 0 0 1 0 0 1 0 0
0 0 0 1 0 1 0 0 0 0 0 1 0 1 0 0 0 0 1 0 1 0 1 0 1 0 0 0 0 1 0 1 0 0 1 0 1 1 1 1 1 1 1 0 0 0 0 0 1 0 1 0 0 0 0 0 0 1 0 0 0 1 0 0 0
0 0 0 1 0 1 0 0 0 0 0 1 0 1 0 0 0 0 1 0 1 0 1 0 1 0 0 0 0 1 0 1 0 0 1 0 0 0 0 0 0 0 1 0 0 0 0 1 0 1 0 0 0 0 0 0 0 0 1 0 1 0 0 0 0
0 0 0 1 0 1 0 0 0 0 0 1 0 1 0 0 0 0 1 0 1 0 1 0 1 0 0 0 0 1 0 1 0 0 1 1 1 1 1 1 1 0 1 0 0 0 1 0 1 0 0 0 0 0 0 0 0 0 1 0 1 0 0 0 0
0 0 0 1 0 1 0 0 0 0 0 1 0 1 0 0 0 0 1 0 1 0 1 0 1 0 0 0 0 1 0 1 0 0 0 0 0 0 0 0 1 0 1 0 0 1 0 1 0 0 0 0 0 0 0 0 0 0 1 0 1 0 0 0 0
1 1 1 1 0 1 1 1 1 0 0 1 0 1 1 1 1 1 1 0 1 0 1 0 1 1 1 1 1 1 0 1 0 0 1 1 1 1 1 1 1 0 1 0 1 0 0 1 1 1 1 1 1 1 0 0 0 0 1 0 1 0 0 0 0
0 0 0 0 0 0 0 0 1 0 0 1 0 0 0 0 0 0 0 0 1 0 1 0 0 0 0 0 0 0 0 1 0 0 1 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 1 0 1 0 0 0 0
0 0 0 0 0 0 0 0 1 1 1 1 0 1 1 1 1 1 1 1 0 0 1 1 1 1 1 1 1 1 1 0 0 0 1 1 1 1 1 1 1 1 1 0 1 0 1 1 1 1 1 1 1 1 0 1 1 1 1 0 1 0 0 0 0    
];
% 1. 找到所有为1的点
[row, col] = find(A == 1);

% 2. 映射到空间坐标
x_range = linspace(0.5, -0.5, size(A,2));
y_range = 0.5;
z_val = linspace(0.6, 0.45, size(A,1));

points = [x_range(col)', y_range*ones(length(row),1),z_val(row)'];
% 3. 最近点优先排序
N = size(points,1);
visited = false(N,1);
order = zeros(N,1);

% 从第一个点开始
order(1) = 1;
visited(1) = true;
for i = 2:N
    last = order(i-1);
    dists = vecnorm(points - points(last,:), 2, 2);
    dists(visited) = inf; % 已访问的点不再考虑
    [~, idx] = min(dists);
    order(i) = idx;
    visited(idx) = true;
end

XYZ_sorted = points(order,:);
XYZ_sorted = [XYZ_sorted; 0.5 0.5 0.45;];
space =[
    0.5 0.5 0.45;
    0.5 0.5 0.6;
    -0.5 0.5 0.6;
    -0.5 0.5 0.45;
    0.5 0.5 0.45;
];
XYZ_sorted =[space; XYZ_sorted];
% 4. 可视化
figure;
plot3(XYZ_sorted(:,1), XYZ_sorted(:,2), XYZ_sorted(:,3), '-o', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor', 'b');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('最近点优先空间路径规划');
axis equal;
grid on;
step = 5; % 每段插值步数（含首尾）
N = size(XYZ_sorted,1);
T_point = zeros(4,4,N);

% 构造每个点的目标位姿（末端z轴垂直向下）
for i = 1:N
    pos = XYZ_sorted(i,:)';
    a = [0; 1; 0];
    n = [0; 1; 0];
    if abs(dot(a, n)) > 0.99
        n = [1; 0; 0];
    end
    o = cross(a, n); o = o / norm(o);
    n = cross(o, a);
    R = [n o a];
    T_point(:,:,i) = [R pos; 0 0 0 1];
end

% 空间插值
T = [];
for i = 1:N-1
    T_start = T_point(:,:,i);
    T_end   = T_point(:,:,i+1);
    T_traj = ctraj(T_start, T_end, step); % 4x4xstep
    T = cat(3, T, T_traj);
end

T = cat(3, T, T_point(:,:,end)); % 补上最后一个点


% 轨迹点数
num_points = size(T,3);

% 逆解工具
ik = inverseKinematics('RigidBodyTree', robot);
weights = [1 1 1 1 1 1];
initialguess = zeros(1,6);

reachable = false(N,1); % N为空间点数

for i = 1:N
    T_now = T_point(:,:,i);
    [~, solInfo] = ik(robot.BodyNames{end}, T_now, weights, initialguess);
    if solInfo.ExitFlag > 0
        reachable(i) = true; % 在工作空间内
    else
        reachable(i) = false; % 不在工作空间内
    end
end

% 可视化结果
figure;
scatter3(XYZ_sorted(reachable,1), XYZ_sorted(reachable,2), XYZ_sorted(reachable,3), 50, 'g', 'filled'); hold on;
scatter3(XYZ_sorted(~reachable,1), XYZ_sorted(~reachable,2), XYZ_sorted(~reachable,3), 50, 'r', 'filled');
xlabel('X'); ylabel('Y'); zlabel('Z');
legend('可达','不可达');
title('空间点可达性判断');
axis equal; grid on;

% 逆解工具已定义：ik, weights, initialguess
W   = zeros(num_points, 6); % 关节角度
Wd  = zeros(num_points, 6); % 关节速度
Wdd = zeros(num_points, 6); % 关节加速度

for i = 1:num_points
    T_now = T(:,:,i);
    [configSol, solInfo] = ik(robot.BodyNames{end}, T_now, weights, initialguess);
    if solInfo.ExitFlag > 0
        W_rad(i,:) = configSol;
        initialguess = configSol;
    else
        % 若不可达，沿用上一个点
        if i > 1
            W_rad(i,:) = W_rad(i-1,:);
        else
            W_rad(i,:) = zeros(1,6);
        end
    end
end
W=rad2deg(W_rad);
% 计算速度和加速度
step_time = 0.05; % 步长，可根据实际调整
t = (0:num_points-1)' * step_time;
Wd  = [zeros(1,6); diff(W)/step_time];
Wdd = [zeros(1,6); diff(Wd)/step_time];

% 重新组织输出，每列为：时间、关节1角度、关节1速度、关节1加速度、关节2角度、关节2速度、关节2加速度、...、关节6角度、关节6速度、关节6加速度
OUTPUT = t;
for j = 1:6
    OUTPUT = [OUTPUT, W(:,j), Wd(:,j), Wdd(:,j)];
end
OUTPUT = OUTPUT(:,1:19);


writematrix(OUTPUT, 'C:\Users\20905\Desktop\output_data2.csv');
