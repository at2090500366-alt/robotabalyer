    a1=0.18;
    a2=0.6;
    a3=0.12;
    a4=0;
    a5=0;
    a6=0;
    d1=0.4; 
    d2=0.135;
    d3=0.135;
    d4=0.62;
    d5= 0;
    d6=0.0; 
    alpha1 = deg2rad(90);
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


% 圆心和半径
x0 = 0.5;      % 平面x=0.5
y0 = 0;        % 圆心y坐标
z0 = 0.2;        % 圆心z坐标
r  = 0.2;      % 圆半径

% 五角星顶点坐标
star_points = zeros(5,3);
for k = 1:5
    angle = pi/2 + (k-1)*2*pi/5; % 从y轴正向逆时针
    star_points(k,1) = x0;
    star_points(k,2) = y0 + r * cos(angle);
    star_points(k,3) = z0 + r * sin(angle);
end

for i = 1:size(star_points,1)
    px = star_points(i,1);
    py = star_points(i,2);
    pz = star_points(i,3);
    m3_1 = (px^2 + py^2 + pz^2 - a2^2 - a3^2 - d2^2 - d4^2) / (2*a2);
    temp = a3^2 + d4^2 - m3_1^2;
    if temp < 0
        fprintf('顶点%d不可达（逆解无实数解）！\n', i);
    else
        fprintf('顶点%d可达。\n', i);
    end
end

disp('五角星顶点坐标:');
for i = 1:size(star_points,1)
    fprintf('%.4f %.4f %.4f\n', star_points(i,1), star_points(i,2), star_points(i,3));
end
%最终版
order = [1 3 5 2 4 1];
% 绘制五角星
figure;
plot3(star_points(order,1), star_points(order,2), star_points(order,3), 'r-', 'LineWidth', 2);
hold on;
scatter3(star_points(:,1), star_points(:,2), star_points(:,3), 'filled', 'MarkerFaceColor', 'b');

% 添加标签和标题
title('三维空间中的五角星');
xlabel('X轴');
ylabel('Y轴');
zlabel('Z轴');
grid on;
axis equal; % 保持各轴比例一致
hold off;




% 假设五角星平面为x=常数，approach向量始终为[1;0;0]（即沿x轴正方向）

Tc = zeros(4,4,5); % 存储五个传递矩阵

for i = 1:5
    % 末端位置
    px = star_points(i,1);
    py = star_points(i,2);
    pz = star_points(i,3);

    % approach向量（末端z轴）设为x轴正方向
    a = [1; 0; 0];

    % n向量任选，只需与a不平行即可，这里取全局z轴
    n = [0; 0; 1];
    if abs(dot(a, n)) > 0.99 % 若a与n接近平行，换一个
        n = [0; 1; 0];
    end

    % o向量（末端y轴）= a × n
    o = cross(a, n);
    o = o / norm(o); % 单位化
    % n重新正交化
    n = cross(o, a);

    % 构造旋转矩阵R = [n o a]
    R = [n o a];

    % 组装齐次变换矩阵
    Tc(:,:,i) = [R [px; py; pz]; 0 0 0 1];
end

% 检查输出
for i = 1:5
    disp(['第', num2str(i), '个点的传递矩阵:']);
    disp(Tc(:,:,i));
end
% 按照给定的顺序排列传递矩阵
Tc_ordered = zeros(4,4,length(order));
for i = 1:length(order)
    Tc_ordered(:,:,i) = Tc(:,:,order(i));
end

disp('按order顺序排列后的五角星传递矩阵：');
for i = 1:length(order)
    disp(['第', num2str(i), '个点:']);
    disp(Tc_ordered(:,:,i));
end

step = 50; % 每段插值的步数，可根据需要调整
N = size(Tc_ordered,3);

% 存储所有插值结果
T_traj_all = [];

for i = 1:N-1
    T_start = Tc_ordered(:,:,i);
    T_end   = Tc_ordered(:,:,i+1);
    % 笛卡尔空间插值
    T_traj = ctraj(T_start, T_end, step); % 4x4xstep
    T_traj_all = cat(3, T_traj_all, T_traj); % 拼接到总轨迹
end

% 检查插值结果
disp(['总共插值步数: ', num2str(size(T_traj_all,3))]);
disp('第一个插值点的传递矩阵:');
disp(T_traj_all(:,:,1));
disp('最后一个插值点的传递矩阵:');
disp(T_traj_all(:,:,end));


M = size(T_traj_all, 3); % 总插值点数
W = zeros(6, M);         % 每列为一个插值点的六个关节角度

ik = inverseKinematics('RigidBodyTree', robot);
weights = [1 1 1 1 1 1];
initialguess = zeros(1,6);

for i = 1:M
    T_goal = T_traj_all(:,:,i);
    [configSol, ~] = ik(robot.BodyNames{end}, T_goal, weights, initialguess);
    W(:,i) = configSol';         % 存储为6×M
    initialguess = configSol;    % 更新初始猜测，保证轨迹连续
end
W_deg = rad2deg(W);
W_deg_T = W_deg';

disp('所有插值点的关节角度（弧度），每列为一个点：');
disp(W);

disp('所有插值点的关节角度（角度），每列为一个点：');
disp(rad2deg(W));


%微分计算速度和加速度
dt = 0.05; % 每个插值点的时间间隔（可根据实际轨迹时长和step调整）

% W_deg_T: 行为插值点，列为关节
Wd = diff(W_deg_T) / dt;           % 速度（角度/秒），(M-1)×6
Wdd = diff(Wd) / dt;               % 加速度（角度/秒^2），(M-2)×6

% 可选：在首行补零，使长度与W_deg_T一致
Wd = [zeros(1, size(Wd,2)); Wd];       % M×6
Wdd = [zeros(2, size(Wdd,2)); Wdd];    % M×6

disp('所有插值点的关节速度（角度/秒）：');
disp(Wd);

disp('所有插值点的关节加速度（角度/秒^2）：');
disp(Wdd);

% 假设W_deg_T, Wd, Wdd已计算，dt已知
time_vec = (0:M-1)' * dt; % M×1，累加时间

% 组合结果矩阵
Result = [time_vec, ...
          W_deg_T(:,1), Wd(:,1), Wdd(:,1), ...
          W_deg_T(:,2), Wd(:,2), Wdd(:,2), ...
          W_deg_T(:,3), Wd(:,3), Wdd(:,3), ...
          W_deg_T(:,4), Wd(:,4), Wdd(:,4), ...
          W_deg_T(:,5), Wd(:,5), Wdd(:,5), ...
          W_deg_T(:,6), Wd(:,6), Wdd(:,6)];


          % 假设 time_vec, W_deg_T, Wd, Wdd 已经计算好

% 角度曲线
figure;
plot(time_vec, W_deg_T);
xlabel('时间 (s)');
ylabel('关节角度 (deg)');
title('各关节角度曲线');
legend('关节1','关节2','关节3','关节4','关节5','关节6');
grid on;

% 速度曲线
figure;
plot(time_vec, Wd);
xlabel('时间 (s)');
ylabel('关节速度 (deg/s)');
title('各关节速度曲线');
legend('关节1','关节2','关节3','关节4','关节5','关节6');
grid on;

% 加速度曲线
figure;
plot(time_vec, Wdd);
xlabel('时间 (s)');
ylabel('关节加速度 (deg/s^2)');
title('各关节加速度曲线');
legend('关节1','关节2','关节3','关节4','关节5','关节6');
grid on;
% 可选：显示前几行

writematrix(Result, 'C:\Users\20905\Desktop\output_data1.csv');

