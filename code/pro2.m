clc
clear
close all

%%

% 读取数据  
data = readtable('A1.csv');  
time = data.time;  
vehicle_ids = unique(data.vehicle_id);  
x = data.x;  
y = data.y;  
figure(6)
plot(x,y,'o')
xlabel('横坐标');
ylabel('纵坐标');
title(['车辆轨迹']);
line = mode(x);  %等红灯的线  
waiting_vehicel = [];
flag = 0;
green_start_last = 0;
T_red_start_last = 0;
% 初始化周期参数  
T_signal = []; % 信号灯周期  red + green
T_red = []; % 红灯时长  
T_green = []; % 绿灯时长  
T_circle = [];% 组合
  
% 设定速度阈值，用于判断车辆是否停下  
speed_threshold = 0.5; % 例如，0.5 m/s  
  
% 对每辆车进行分析  
for i = 1:length(vehicle_ids)
    idx = data.vehicle_id == vehicle_ids(i);  %找出特定编号的车
    vehicle_x = x(idx);
    vehicle_y = y(idx);
    vehicle_time = time(idx);
    % 计算速度  
    vehicle_dx = diff(vehicle_x);  
    vehicle_dy = diff(vehicle_y);  
    vehicle_dt = diff(vehicle_time);  
    vehicle_speed = sqrt(vehicle_dx.^2 + vehicle_dy.^2) ./ vehicle_dt;  

    % 找到车辆停止等待红灯的点  
    stop_indices = find(vehicle_speed < speed_threshold);  
    %停车时间
    % stop_time = vehicle_time(stop_indices)
    %flag =flag +1

    % 如果没有找到停止的点，则跳过此车辆  
    
    if isempty(stop_indices)  
        continue;  
    end  

    % 如果车距离信号灯太远，则跳过
    if vehicle_x(stop_indices(1)) <(line-10) && vehicle_x(stop_indices(1)) >(line+10)
        continue;  
    end  
    % 假设第一辆停下的车开始的时间为红灯开始时间  
     
    T_red_start = vehicle_time(stop_indices(1));  

    % 假设车辆再次开始移动时为绿灯开始时间  
    green_start = vehicle_time(stop_indices(end) + 1); 

    
    %淘汰比第一辆停下的车晚来的车
    if  i >1 && T_red_start > T_red_start_last && T_red_start < green_start_last 
        continue;  
    end  
    
    T_red_current = green_start - T_red_start;  
    T_green_current = green_start - green_start_last - T_red_current;
    % 筛选每个周期第一辆停下的车
    % T_red = [T_red, T_red_current]; 
    % T_green = [T_green, T_green_current];  
    % 如果停止时间太短的点，则跳过此车辆  
    if  length(stop_indices)<10
        green_start = green_start_last;
        continue;  
    end 

    for j = 1:length(vehicle_time)
        % 创建逻辑数组，标记同时满足两个条件的行  
        logical_idx = (data.time == vehicle_time(j)) & (data.vehicle_id == vehicle_ids(i));  
  
        % 使用 find 函数找到这些逻辑为真的行的索引  
        row_indices = find(logical_idx);
        if j == 1
            data.v(row_indices) = vehicle_speed(1);
        else
        data.v(row_indices) = vehicle_speed(j-1);
        end
    end
    
    if  i >1 && T_red_start < T_red_start_last 
        T_signal = [T_signal(1:end-1) , green_start - green_start_last];
       T_red = [T_red(1:end-1), T_red_current];  
       T_green = [T_green(1:end-1), T_green_current];  
       waiting_vehicel = [waiting_vehicel(1:end-1),vehicle_ids(i)];
    else
    % 更新红灯时长（第一个停下到再次移动的时间差）  
    T_signal = [T_signal , green_start - green_start_last];

    T_red = [T_red, T_red_current];
    % 更新绿灯时长（如果有多辆车停下，取最后一个停下到第一个再次移动的时间差）  
    T_green = [T_green, T_green_current];  
    
    
    waiting_vehicel = [waiting_vehicel,vehicle_ids(i)];
    end   
   
    % 更新信号灯周期的开始时间为下一个红灯的开始时间  
    green_start_last = green_start;
    T_red_start_last = T_red_start;  
    
end  

%每个周期第一辆停下的车所停时间
T_waiting_vehicel = horzcat(waiting_vehicel',T_red');

% 估计信号灯周期  

t =zeros(length(T_red),1);
for i = 1:length(T_red )
    t(i) = i;
end
T_circle = [t,(T_signal)',T_red',T_green'];  
T_circle = array2table(T_circle);

%画出每个周期第一辆停下的车的轨迹图
figure(1)
    hold on;
    n =size(T_waiting_vehicel);
    for j = 1:n(1)
        vehicle_data = data(data.vehicle_id == T_waiting_vehicel(j,1), :);
        plot(vehicle_data.time, vehicle_data.x, '-', 'DisplayName', ['Vehicle ' num2str(T_waiting_vehicel(j,1))]);
    end
    
    xlabel('时间(s)');
    ylabel('车辆坐标(m)');
    title(['车辆坐标-时间图']);
    legend('show');
    
    hold off;
%速度-时间图
figure(5)
    hold on;
    n =size(T_waiting_vehicel);
    for j = 1:n(1)
        vehicle_data = data(data.vehicle_id == T_waiting_vehicel(j,1), :);
        plot(vehicle_data.time, vehicle_data.v, '-', 'DisplayName', ['Vehicle ' num2str(T_waiting_vehicel(j,1))]);
    end
    
    xlabel('时间(s)');
    ylabel('车辆速度（m/s）');
    ylim([-5,20]);
    title(['车辆时间-速度图']);
    legend('show');
    
    hold off;
    
    % 初始化 C 为一个空矩阵  
C = [];  
  
% 遍历 A 和 B 的每个元素  
for i = 1:length(T_red)  
    % 将 A(i) 个 1 添加到 C 中  
    C = [C; ones(T_green(i), 1)];  
    % 将 B(i) 个 0 添加到 C 中（如果 B(i) 不为 0）  
    C = [C; zeros(T_red(i), 1)];  
     
end  

%处理前
figure(2)
plot(C)

ylim([-0.5 1.5]);  
xlabel('时间');
ylabel('value')
title(['红绿灯周期图，1-绿，0-红（处理前）']);
hold off
%%

% 数据处理->删除离群值
T_circle = rmoutliers(T_circle,"quartiles", ...
	    "DataVariables",["T_circle2","T_circle3","T_circle4"]);
T_circle = table2array(T_circle);
T_signal = T_circle(:,2);
T_red = (T_circle(:,3));
T_green = (T_circle(:,4));

% 显示结果  
disp(['估计的信号灯周期（秒）: ', num2str(mean(T_signal(:,1)))]);  
disp(['估计的红灯时长（秒）: ', num2str(mode(T_red))]);  
disp(['估计的绿灯时长（秒）: ', num2str(mean(T_signal(:,1))-mode(T_red))]);

C = [];  
  
% 遍历 A 和 B 的每个元素  
for i = 1:length(T_red)  
    % 将 A(i) 个 1 添加到 C 中  
    C = [C; ones(T_green(i), 1)];  
    % 将 B(i) 个 0 添加到 C 中（如果 B(i) 不为 0）  
    C = [C; zeros(T_red(i), 1)];  
     
end  
figure(3)
plot(C)

ylim([-0.5 1.5]);  
xlabel('时间');
ylabel('value')
title(['红绿灯周期图，1-绿，0-红(处理后)']);
hold off


%%
% 画图
figure(4)
data1 = table2array(data);
num_vehicle_ids = length(vehicle_ids);
max_legend_entries = 21; % 每个图例中的最大条目数

num_subplots = ceil(num_vehicle_ids / max_legend_entries); % 计算需要的子图数量

for subplot_index = 1:num_subplots
    start_index = (subplot_index - 1) * max_legend_entries + 1;
    end_index = min(subplot_index * max_legend_entries, num_vehicle_ids);
    
    subplot(num_subplots, 1, subplot_index);
    hold on;
    
    for i = start_index:end_index
        vehicle_data = data(data.vehicle_id == vehicle_ids(i), :);
        plot(vehicle_data.time, vehicle_data.y, '-', 'DisplayName', ['Vehicle ' num2str(vehicle_ids(i))]);
    end
    
    xlabel('时间');
    ylabel('车辆坐标');
    title(['车辆坐标-时间图(第' num2str(subplot_index) '组']);
    %legend('show');
    
    hold off;
end

