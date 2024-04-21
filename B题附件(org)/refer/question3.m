clear;clc
% data = readmatrix("C1.csv");
% data = readmatrix("C2.csv");
% data = readmatrix("C3.csv");
% data = readmatrix("C4.csv");
% data = readmatrix("C5.csv");
data = readmatrix("C5.csv");

%% 参数
speed_threshold = 0.5; % 设定速度变化阈值（单位：m/s）
heading_change_threshold = 15; % 设定航向角变化阈值（单位：度）
buffer_radius = 5; % 缓冲区半径，可以根据实际情况调整
stop_speed_threshold = 0.1; % 停止速度阈值（单位：m/s），用于确定车辆是否停止
threshold = 20; % 周期时长变化阈值

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
% title('C1');
%% 根据停止和启动时间估计信号灯周期和时长
all_stop_events = sort(all_stop_events);
all_start_events = sort(all_start_events);

% 初始化变量
current_cycle_start = all_stop_events(1);
cycle_times = [];
red_light_times = [];
green_light_times = [];
cycle_switch_times = []; % 用于存储周期切换的时间
old_cycle_time = NaN; % 用于存储上一个周期的时长

% 遍历停止事件
for i = 1:length(all_stop_events)
    current_stop_event = all_stop_events(i);

    % 找到所有在当前停止事件之后的启动事件
    future_start_events = all_start_events(all_start_events > current_stop_event);

    if isempty(future_start_events)
        continue; % 如果没有未来的启动事件，则跳过当前循环
    end

    % 找到紧接在当前停止事件之后的启动事件
    next_start_event = future_start_events(1);

    % 找到当前周期内的最后一个停止事件
    last_stop_idx = find(all_stop_events > current_stop_event & all_stop_events < next_start_event, 1, 'last');

    if isempty(last_stop_idx)
        continue; % 如果没有找到最后一个停止事件，则跳过当前循环
    end

    % 查找当前周期内的第一个启动事件
    first_start_idx = find(all_start_events > current_stop_event, 1, 'first');

    if isempty(last_stop_idx) || isempty(first_start_idx)
        continue; % 如果没有找到对应的事件，则跳过当前循环
    end

    last_stop_event = all_stop_events(last_stop_idx);
    first_start_event = all_start_events(first_start_idx);

    % 计算当前周期时长
    current_cycle_time = first_start_event - current_stop_event;

    % 检查周期是否切换
    if ~isnan(old_cycle_time) && abs(current_cycle_time - old_cycle_time) > threshold
        % 周期切换，记录切换时间和旧周期时长
        cycle_switch_times(end+1) = current_stop_event;
        old_cycle_time = current_cycle_time; % 更新旧周期时长为当前周期时长
    end

    % 计算红灯时长
    current_red_light_time = next_start_event - current_stop_event;
    red_light_times(end+1) = current_red_light_time;

    % 计算绿灯时长（假设到下一个周期开始之前都是绿灯，或者没有下一个周期开始事件时结束）  
    next_cycle_start_idx = find(all_stop_events > first_start_event, 1, 'first');  
    if ~isempty(next_cycle_start_idx)  
        next_cycle_start_event = all_stop_events(next_cycle_start_idx);  
        current_green_light_time = next_cycle_start_event - first_start_event;  
    else  
        current_green_light_time = NaN; % 如果没有下一个周期的开始事件，则绿灯时长为NaN  
    end  
    green_light_times(end+1) = current_green_light_time; 

    % 更新当前周期开始时间
    current_cycle_start = first_start_event;

    % 记录当前周期时长
    cycle_times(end+1) = current_cycle_time;
    old_cycle_time = current_cycle_time; % 更新旧周期时长
end

% 过滤掉非正值和NaN值
valid_cycle_times = cycle_times(cycle_times > 0 & ~isnan(cycle_times));
valid_red_light_times = red_light_times(red_light_times > 0 & ~isnan(red_light_times));
valid_green_light_times = green_light_times(green_light_times > 0 & ~isnan(green_light_times));

% 初始化周期平均时长数组
average_cycle_times = [];
average_red_light_times = [];
average_green_light_times = [];

% 遍历所有周期切换时间点，除了最后一个（因为没有后续时间点来确定周期结束）
for i = 1:(length(cycle_switch_times) - 1)
    % 提取当前周期的信号灯时长
    current_cycle_times = valid_cycle_times(i+1); % 注意索引从i+1开始，因为valid_cycle_times的第一个元素对应cycle_switch_times的第一个周期之后的周期
    current_red_light_times = valid_red_light_times(i+1);
    current_green_light_times = valid_green_light_times(i+1);

    % 如果周期时长、红灯时长或绿灯时长是NaN，则忽略当前周期
    if isnan(current_cycle_times) || isnan(current_red_light_times) || isnan(current_green_light_times)
        continue; % 跳过当前循环，继续下一个周期
    end

    % 假设每个周期只有一个时长，所以直接计算平均值
    average_cycle_times(end+1) = current_cycle_times; % 假设只有一个值，直接添加
    average_red_light_times(end+1) = current_red_light_times;
    average_green_light_times(end+1) = current_green_light_times;
end

% 初始化一个结构体数组来存储每个周期的信息
cycles = struct();
num_cycles = length(cycle_switch_times);
for i = 1:num_cycles
    cycles(i).cycle_time = valid_cycle_times(i);
    cycles(i).red_light_time = valid_red_light_times(i);

    % 如果没有对应的绿灯时长，则可能需要计算或设为NaN
    if ~isnan(valid_green_light_times(i))
        cycles(i).green_light_time = valid_green_light_times(i);
    else
        % 这里可能需要一些逻辑来计算绿灯时长，或者将其设为NaN
        cycles(i).green_light_time = NaN;
    end

    % 查找当前周期的切换时间（如果有的话）
    if ~isempty(cycle_switch_times) && i <= length(cycle_switch_times)
        cycles(i).switch_time = cycle_switch_times(i);
    else
        cycles(i).switch_time = NaN; % 如果没有周期切换时间，则设为NaN
    end
end

% 现在，您可以遍历cycles结构体数组来分析每个周期的信息
for i = 1:num_cycles
    fprintf('Cycle %d:\n', i);
    fprintf('  Cycle Time: %.2f\n', cycles(i).cycle_time);
    fprintf('  Red Light Time: %.2f\n', cycles(i).red_light_time);
    fprintf('  Green Light Time: %.2f\n', cycles(i).green_light_time);
    fprintf('  Switch Time: %.2f\n', cycles(i).switch_time);
    fprintf('\n');
end