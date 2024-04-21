clear;clc
% data = readmatrix("B1.csv");
% data = readmatrix("B2.csv");
% data = readmatrix("B3.csv");
% data = readmatrix("B4.csv");
data = readmatrix("B5.csv");

%% 参数
speed_threshold = 1; % 设定速度变化阈值（单位：m/s）
heading_change_threshold = 15; % 设定航向角变化阈值（单位：度）
buffer_radius = 5; % 缓冲区半径，可以根据实际情况调整
stop_speed_threshold = 0.1; % 停止速度阈值（单位：m/s），用于确定车辆是否停止

%% 获得停止和启动时间
% 提取唯一的vehicle_id
unique_vehicle_ids = unique(data(:, 2));

% 初始化cell数组来存储每个vehicle_id的候选交叉口位置和停止/启动事件
intersection_candidates_cell = cell(numel(unique_vehicle_ids), 1);
stop_start_events_cell = cell(numel(unique_vehicle_ids), 1);

% 初始化停止和启动事件数组
all_stop_events = [];
all_start_events = [];

% 循环遍历每个vehicle_id
for i = 1:numel(unique_vehicle_ids)
    current_vehicle_id = unique_vehicle_ids(i);

    % 提取当前vehicle_id的所有数据
    vehicle_indices = find(data(:, 2) == current_vehicle_id);
    vehicle_data = data(vehicle_indices, :);

    % 提取x和y坐标以及时间
    time_vector = vehicle_data(:, 1);
    x_vector = vehicle_data(:, 3);
    y_vector = vehicle_data(:, 4);
    plot(x_vector,y_vector,'.');
    hold on;
    % 计算速度
    dx = diff(x_vector);
    dy = diff(y_vector);
    dt = diff(time_vector);
    speeds = sqrt(dx.^2 + dy.^2) ./ dt;
    speeds = [NaN; speeds]; % 在速度数组前添加一个NaN以匹配原始数据长度

    % 识别速度变化点
    speed_change_indices = find(abs(diff(speeds)) > speed_threshold);

    % 计算航向角
    headings = atan2(diff(y_vector), diff(x_vector));
    headings_deg = rad2deg(headings);

    % 识别转向事件
    heading_change_indices = find(abs(diff(headings_deg)) > heading_change_threshold);

    % 交叉验证：寻找同时满足速度变化和转向事件的点
    intersection_indices = intersect(speed_change_indices, heading_change_indices);

    % 提取候选交叉口位置
    if ~isempty(intersection_indices)
        intersection_candidates = [time_vector(intersection_indices + 1), x_vector(intersection_indices + 1), y_vector(intersection_indices + 1)];
    else
        intersection_candidates = [];
    end

    % 存储候选交叉口位置到cell数组
    intersection_candidates_cell{i} = intersection_candidates;

    % 识别停止和启动事件
    stop_idx = find(speeds < stop_speed_threshold);
    start_idx = find(diff([false; speeds >= stop_speed_threshold]));

    stop_idx = stop_idx(stop_idx < length(time_vector) - 1); % 过滤出有效的stop_idx  
    start_idx = start_idx(start_idx < length(time_vector) - 1); % 过滤出有效的start_idx 

    % 提取停止和启动事件的时间、x坐标和y坐标
    stop_events = [time_vector(stop_idx + 1), x_vector(stop_idx + 1), y_vector(stop_idx + 1)];
    start_events = [time_vector(start_idx + 1), x_vector(start_idx + 1), y_vector(start_idx + 1)];

    % 确定停止和启动事件是否在候选交叉口缓冲区内
    if ~isempty(intersection_candidates)
        intersection_x = intersection_candidates(:, 2);
        intersection_y = intersection_candidates(:, 3);

        in_intersection_stop_idx = zeros(size(stop_events, 1), 1);
        for j = 1:size(stop_events, 1)
            if sqrt((stop_events(j, 2) - intersection_x).^2 + (stop_events(j, 3) - intersection_y).^2) <= buffer_radius^2
                in_intersection_stop_idx(j) = 1;
            end
        end

        % 在缓冲区半径内检查启动事件
        in_intersection_start_idx = zeros(size(start_events, 1), 1);
        for j = 1:size(start_events, 1)
            if sqrt((start_events(j, 2) - intersection_x).^2 + (start_events(j, 3) - intersection_y).^2) <= buffer_radius^2
                in_intersection_start_idx(j) = 1;
            end
        end
        % 将整数索引数组转换为逻辑数组
        logical_stop_idx = in_intersection_stop_idx ~= 0;
        logical_start_idx = in_intersection_start_idx ~= 0;
        % 提取交叉口附近的停止和启动事件
        in_intersection_stop_events = stop_events(logical_stop_idx, :);
        in_intersection_start_events = start_events(logical_start_idx, :);

        % 将当前车辆的停止和启动事件添加到总数组中
        all_stop_events = [all_stop_events; in_intersection_stop_events(:, 1)]; % 只存储时间
        all_start_events = [all_start_events; in_intersection_start_events(:, 1)]; % 只存储时间
        % 存储当前车辆的停止和启动事件到cell数组
        stop_start_events_cell{i} = {in_intersection_stop_events, in_intersection_start_events};
    end
end
% hold off;
% title('B1');
%% 根据停止和启动时间估计信号灯周期和时长
all_stop_events = sort(all_stop_events);
all_start_events = sort(all_start_events);

% 初始化周期、红灯和绿灯时长的数组
cycle_times = [];
red_light_times = [];
green_light_times = [];

% 遍历停止事件来识别周期的开始
for i = 1:length(all_stop_events)
    % 假设当前停止事件是一个周期的开始
    current_cycle_start = all_stop_events(i);

    % 查找当前周期内的最后一个停止事件和第一个启动事件
    last_stop_idx = find(all_stop_events > current_cycle_start, 1, 'first');
    if isempty(last_stop_idx)
        continue; % 如果没有找到停止事件，则跳过当前循环
    end
    last_stop_event = all_stop_events(last_stop_idx);

    first_start_idx = find(all_start_events > last_stop_event, 1, 'first');
    if isempty(first_start_idx)
        continue; % 如果没有找到启动事件，则跳过当前循环
    end
    first_start_event = all_start_events(first_start_idx);

    % 计算周期时长
    current_cycle_time = first_start_event - current_cycle_start;
    cycle_times(end+1) = current_cycle_time;

    % 计算红灯时长（假设最后一个停止事件到第一个启动事件之间是红灯）
    current_red_light_time = first_start_event - last_stop_event;
    red_light_times(end+1) = current_red_light_time;

    % 计算绿灯时长（假设第一个启动事件到下一个周期的开始之间是绿灯）
    % 注意：这里假设没有黄灯时间或者行人过街时间，实际情况可能更复杂
    next_cycle_start_idx = find(all_stop_events > first_start_event, 1, 'first');
    if ~isempty(next_cycle_start_idx)
        next_cycle_start_event = all_stop_events(next_cycle_start_idx);
        current_green_light_time = next_cycle_start_event - first_start_event;
    else
        % 如果没有下一个周期的开始事件，则绿灯时长为NaN
        current_green_light_time = NaN;
    end
    green_light_times(end+1) = current_green_light_time;
end

% 过滤掉非正值和非NaN值
valid_cycle_times = cycle_times(cycle_times > 0 & ~isnan(cycle_times));
valid_red_light_times = red_light_times(red_light_times > 0 & ~isnan(red_light_times));
valid_green_light_times = green_light_times(green_light_times > 0 & ~isnan(green_light_times));


% 计算平均值
estimated_cycle_time = mean(valid_cycle_times);
estimated_red_light_time = mean(valid_red_light_times);
estimated_green_light_time = mean(valid_green_light_times);

% 显示结果
disp(['平均循环时间:', num2str(estimated_cycle_time)]);
disp(['平均红灯时长:', num2str(estimated_red_light_time)]);
disp(['平均绿灯时长:', num2str(estimated_green_light_time)]);
