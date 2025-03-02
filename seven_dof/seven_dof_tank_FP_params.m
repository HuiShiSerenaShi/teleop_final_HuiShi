%% params_7dof.m
% Sample parameters for "Tank-based 7DoF Teleoperation"
clear all;
close all;
clc;

%(1) 通用参数：采样、噪声、延迟
s = tf('s');
Ts   = 0.001;     % 仿真采样时间
d    = 10;        % 传输延时(单位:离散步)


%(6) 其他1DoF遗留参数

% Ph  = 10;  % 人意图PD增益(1DoF) -> 7×7 diag
% % Dh  = 16;  % (20*0.8)
% Ph  = 2;  % 人意图PD增益(1DoF) -> 7×7 diag
% Dh  = 4;  % (20*0.8)
Ph  = 2;  % 人意图PD增益(1DoF) -> 7×7 diag
Dh  = 4;  % (20*0.8)

Flp = 5;   % 在Zh里做低通时使用
% (2) 人机 & 机器臂动力学参数 (保持1DoF风格, 简化为标量/对角)
% 人机阻抗
Jh  = 0;
Bh  = 2;
Kh  = 0;

% Tank / Passivity 相关
tlcAlpha = 0.5;   % Tank Level Controller增益
beta     = 0.1; % 能量发送比例
Hd       = 0.1;   % 期望罐能量
Hm_init  = Hd*20;   % 主侧罐 初始能量
Hs_init  = Hd*20;   % 从侧罐 初始能量


% % 从侧控制增益
% Bs = 10;     % 128
% Ks = 50;     % 40
% 从侧控制增益
% Bs = 15;     % 128
% Ks = 60;     % 40

Bs = 15;     % 128
Ks = 60;     % 40

% 每个关节独立的环境参数
% Ke = [200; 150; 180; 210; 190; 220; 200]; % 刚度 (N/m)
% Be = [50; 40; 45; 55; 50; 60; 50];       % 阻尼系数 (Ns/m)
% xe = [0.5; 0.6; 0.4; 0.7; 0.8; 0.3; 0.9]; % 环境位置 (m)
Ke = 200 * ones(7, 1); % 所有关节刚度相同
Be = 100 * ones(7, 1);  % 所有关节阻尼系数相同
% xe = 2 * ones(7, 1); % 所有关节环境位置相同
xe = [4; 4; 4; 4; 4; 4; 4]; % 环境位置 (m)
xe = [1; 1; 1; 1; 1; 1; 1]; % 环境位置 (m)

q0_s  = [0.0167305,-0.762614,-0.0207622,-2.34352,-0.0305686,1.53975,0.753872]'; % 行向量
dq0_s = zeros(7, 1);           % 行向量

q0_m =  [0.0167305,-0.762614,-0.0207622,-2.34352,-0.0305686,1.53975,0.753872]';
dq0_m = zeros(7, 1); 
qend_m = q0_m; % 从初始值扩展目标值
qend_m(1) = q0_m(1) + 1.2; % 根据任务需求调整各关节
qend_m(2) = q0_m(2) - 0.5;
qend_m(3) = q0_m(3) - 0.7;
qend_m(4) = q0_m(4) + 1;
qend_m(5) = q0_m(5) - 1;
qend_m(6) = q0_m(6) - 1;
qend_m(7) = q0_m(7) + 1.5;
% qend_m(1) = q0_m(1) + 0.6; % 原为 1.2
% qend_m(2) = q0_m(2) - 0.25; % 原为 -0.5
% qend_m(3) = q0_m(3) - 0.35; % 原为 -0.7
% qend_m(4) = q0_m(4) + 0.5; % 原为 1
% qend_m(5) = q0_m(5) - 0.5; % 原为 -1
% qend_m(6) = q0_m(6) - 0.5; % 原为 -1
% qend_m(7) = q0_m(7) + 0.75; % 原为 1.5

t = 0:Ts:20; % 仿真时间
n_steps = length(t); % 时间步数

% 初始化 q_desired 矩阵，大小为 [n_steps x 7]
q_desired = zeros(n_steps, 7);

% 为每个关节生成轨迹
for i = 1:7
    q_desired(:, i) = linspace(q0_m(i), qend_m(i), n_steps); % 第 i 个关节的轨迹
end

% 转换为 Simulink 可用的数据格式
sim_input.time = t'; % 时间列向量
sim_input.signals.values = q_desired; % 关节轨迹矩阵，每列对应一个关节
sim_input.signals.dimensions = 7; % 7个关节

LTI_Matrix = eye(7);

set_param(bdroot, 'ShowLineDimensions', 'on');

disp('=== Loaded 7DoF parameters successfully ===');

%% results
time = out.xds.Time; % 时间
xds_data = out.xds.Data; % 数据: xds
xm_data = out.xm.Data;   % 数据: xm
xs_data = out.xs.Data;   % 数据: xs
xe_data = out.xe.Data;   % 数据: xe, 固定值

% 绘图
figure;
tiledlayout(4, 2); % 设置4行2列的布局
sgtitle('Position Variables for 7 DOFs');
for i = 1:7 % 对每个自由度绘图
    nexttile; % 移动到下一个格子
    plot(time, xds_data(:, i), 'b-', 'LineWidth', 1.5); % xds 曲线
    hold on;
    plot(time, xm_data(:, i), 'r-', 'LineWidth', 1.5); % xm 曲线
    plot(time, xs_data(:, i), 'g-', 'LineWidth', 1.5); % xs 曲线
    plot(time, ones(size(time)) * xe_data(i), 'k--', 'LineWidth', 1.5); % 固定 xe 值
    hold off;
    legend({'xds', 'xm', 'xs', 'xe'}, 'Location', 'best');
    title(['DOF ', num2str(i)]);
    xlabel('Time (s)');
    ylabel('Value');
end

% 隐藏空白格子（第8个格子）
if 8 > 7
    nexttile(8);
    axis off;
end


% 速度变量 (dot_xm 和 dot_xs)
dot_xm_data = out.dot_xm.Data;
dot_xs_data = out.dot_xs.Data;

figure;
tiledlayout(4, 2); % 4 行 2 列布局
sgtitle('Velocity Variables for 7 DOFs');
for i = 1:7
    nexttile;
    plot(time, dot_xm_data(:, i), 'r-', 'LineWidth', 1.5); % dot_xm
    hold on;
    plot(time, dot_xs_data(:, i), 'g-', 'LineWidth', 1.5); % dot_xs
    hold off;
    legend({'dot\_xm', 'dot\_xs'}, 'Location', 'best');
    title(['DOF ', num2str(i)]);
    xlabel('Time (s)');
    ylabel('Value');
end
disp('=== 速度变量绘图完成 ===');

% 罐能量变量 (Hm, Hs)
Hm_data = out.Hm.Data;
Hs_data = out.Hs.Data;

figure;
tiledlayout(4, 2); % 4 行 2 列布局
sgtitle('Energy Variables for 7 DOFs');
for i = 1:7
    nexttile;
    plot(time, Hm_data(:, i), 'r-', 'LineWidth', 1.5); % Hm
    hold on;
    plot(time, Hs_data(:, i), 'g-', 'LineWidth', 1.5); % Hs
    hold off;
    legend({'Hm', 'Hs'}, 'Location', 'best');
    title(['DOF ', num2str(i)]);
    xlabel('Time (s)');
    ylabel('Value');
end
disp('=== 罐能量变量绘图完成 ===');

% 力变量 (fh, fe, fcm, fcs, fh_star)
fh_time = out.fh.Time; % 取 fh 对应的时间，假设所有力变量的时间戳一致
fh_data = out.fh.Data;
fe_data = out.fe.Data;
fcm_data = out.fcm.Data;
fcs_data = out.fcs.Data;
fh_star_data = out.fh_star.Data;

figure;
tiledlayout(4, 2); % 4 行 2 列布局
sgtitle('Force Variables for 7 DOFs');
for i = 1:7
    nexttile;
    plot(fh_time, fh_data(:, i), 'b-', 'LineWidth', 1.5); % fh
    hold on;
    plot(fh_time, fe_data(:, i), 'k--', 'LineWidth', 1.5); % fe
    plot(fh_time, fcm_data(:, i), 'r-', 'LineWidth', 1.5); % fcm
    plot(fh_time, fcs_data(:, i), 'g-', 'LineWidth', 1.5); % fcs
    plot(fh_time, fh_star_data(:, i), 'm-', 'LineWidth', 1.5); % fh_star
    hold off;
    legend({'fh', 'fe', 'fcm', 'fcs', 'fh\_star'}, 'Location', 'best');
    title(['DOF ', num2str(i)]);
    xlabel('Time (s)');
    ylabel('Value');
end
disp('=== 力变量绘图完成 ===');



% % 打印所有相关变量的大小
% disp('=== 打印所有变量的大小 ===');
% disp(['time: ', num2str(size(out.xds.Time))]);
% disp(['xds_data: ', num2str(size(out.xds.Data))]);
% disp(['xm_data: ', num2str(size(out.xm.Data))]);
% disp(['xs_data: ', num2str(size(out.xs.Data))]);
% disp(['xe_data: ', num2str(size(out.xe.Data))]);
% disp(['dot_xm_data: ', num2str(size(out.dot_xm.Data))]);
% disp(['dot_xs_data: ', num2str(size(out.dot_xs.Data))]);
% disp(['fh_data: ', num2str(size(out.fh.Data))]);
% disp(['fe_data: ', num2str(size(out.fe.Data))]);
% disp(['fcm_data: ', num2str(size(out.fcm.Data))]);
% disp(['fcs_data: ', num2str(size(out.fcs.Data))]);
% disp(['fh_star_data: ', num2str(size(out.fh_star.Data))]);
% disp(['Hm_data: ', num2str(size(out.Hm.Data))]);
% disp(['Hs_data: ', num2str(size(out.Hs.Data))]);
% disp('=== 打印完成 ===');
% === 打印所有变量的大小 ===
% time: 20001      1
% xds_data: 20001      7
% xm_data: 20001      7
% xs_data: 20001      7
% xe_data: 1  7
% dot_xm_data: 20001      7
% dot_xs_data: 20001      7
% fh_data: 50593      7
% fe_data: 50593      7
% fcm_data: 50593      7
% fcs_data: 50593      7
% fh_star_data: 50593      7
% Hm_data: 20001      7
% Hs_data: 20001      7
% === 打印完成 ===
