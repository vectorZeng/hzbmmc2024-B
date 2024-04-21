clear;
clc
data = readtable("D.csv");

% 删除异常值
data(data.vehicle_id == 5428,:) = [];
 data(data.vehicle_id == 5431,:) = [];
  data(data.vehicle_id == 5468,:) = [];
%% 参数
speed_threshold = 1; % 设定速度变化阈值（单位：m/s）
heading_change_threshold = 30; % 设定航向角变化阈值（单位：度）
buffer_radius = 10; % 缓冲区半径，可以根据实际情况调整
stop_speed_threshold = 0.1; % 停止速度阈值（单位：m/s），用于确定车辆是否停止
threshold = 10; % 周期时长变化阈值

%% 聚类数据准备
% 特征提取
% 提取唯一的vehicle_id
unique_vehicle_ids = unique(data.vehicle_id);

% 初始化cell数组来存储每个vehicle_id的候选交叉口位置和停止/启动事件
intersection_candidates_cell = cell(numel(unique_vehicle_ids), 1);
stop_start_events_cell = cell(numel(unique_vehicle_ids), 1);

% 初始化停止和启动事件数组
all_stop_events = [];
all_start_events = [];
%停下来的车及其停止启动时间
all_start_car = [];
t = zeros(size(unique_vehicle_ids,1),6);

% 循环遍历每个vehicle_id
for i = 1:numel(unique_vehicle_ids)
    current_vehicle_id = unique_vehicle_ids(i);

    % 提取当前vehicle_id的所有数据
    vehicle_indices = find(data.vehicle_id == current_vehicle_id);
    vehicle_data = data(vehicle_indices, :);

    % 提取x和y坐标以及时间
    time_vector = vehicle_data.time;
    x_vector = vehicle_data.x;
    y_vector = vehicle_data.y;

    % 计算速度
    dx = diff(x_vector);
    dy = diff(y_vector);
    dt = diff(time_vector);
    speeds = sqrt(dx.^2 + dy.^2) ./ dt;
    speeds = [NaN; speeds]; % 在速度数组前添加一个NaN以匹配原始数据长度

    % for j = 1:length(time_vector)
    %     % 创建逻辑数组，标记同时满足两个条件的行  
    %     logical_idx = (data.time == time_vector(j)) & (data.vehicle_id == unique_vehicle_ids(i));  
    % 
    %     % 使用 find 函数找到这些逻辑为真的行的索引  
    %     row_indices = find(logical_idx);
    % 
    %     data.v(row_indices) = speeds(j);
    % 
    % end

    % 识别速度变化点
    %加速
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
    %intersection_candidates_cell{i} = intersection_candidates;

    % 特征向量
    t(i,:) = [min(x_vector),max(x_vector),min(y_vector),max(y_vector),max(headings_deg),min(headings_deg)];
    % 识别停止和启动事件
    stop_idx = find(speeds < stop_speed_threshold);
    start_idx = find(diff([false; speeds >= stop_speed_threshold]));
    if isempty(stop_idx)  
        continue;  
    end  
    stop_idx = stop_idx(stop_idx < length(time_vector) - 1); % 过滤出有效的stop_idx
    start_idx = start_idx(start_idx < length(time_vector) - 1); % 过滤出有效的start_idx

    % 
    % red_start = time_vector(stop_idx(1));  
    % 
    % % 假设车辆再次开始移动时为绿灯开始时间  
    % green_start = time_vector(stop_idx(end) + 1); 

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
        %all_stop_events = [all_stop_events; in_intersection_stop_events(:, 1)]; % 只存储时间
        %all_start_events = [all_start_events; in_intersection_start_events(:, 1)]; % 只存储时间
        if ~isempty(in_intersection_stop_events)
            all_start_car = [all_start_car;[unique_vehicle_ids(i),in_intersection_stop_events(1,1),in_intersection_stop_events(end,1)+1]];
        end
        % 存储当前车辆的停止和启动事件到cell数组
        %stop_start_events_cell{i} = {in_intersection_stop_events, in_intersection_start_events};
    end
end
%% k聚类


% k_means
means = mean(t); %求均值
stds = std(t);   %求标准差
data1 = (t-means)./stds; %标准化

%判断k值
K = 15;A = zeros(K,2);
for k=1:K
    [~,~,sumd,~] = kmeans(data1,k);
    sse=sum(sumd.^2);
    A(k,1) = k;
    A(k,2) = sse;
end
figure(1);
plot(A(:,1),A(:,2));
hold on
plot(A(:,1),A(:,2),'ro');
title('不同k值聚类偏差图');
xlabel('k值');ylabel('簇内误差平方和');

%统计项目数
k = 12;
[idx,C,sumd,D] = kmeans(data1,k);

%%
%聚类 12图

% 创建 4x3 的子图网格  
nRows = 4;  
nCols = 3;  
figure; % 创建一个新的图形窗口  
  
% 初始化子图索引  
subplotIndex = 1;  
  
for j = 1:12 % 假设您只需要处理前12个索引  
    Idx = find(idx == j); % qf_Idx 需要在此前定义，并且其长度至少为12  
    unique_vehicle_id = unique_vehicle_ids(Idx);  
   
    for i = 1:numel(unique_vehicle_id)  
        current_vehicle_id = unique_vehicle_id(i);  
          
        % 提取当前 vehicle_id 的所有数据  
        vehicle_indices = find(data.vehicle_id == current_vehicle_id);  
        vehicle_data = data(vehicle_indices, :);  
          
        % 提取 x 和 y 坐标以及时间（时间在这里可能不需要用于绘图）  
        x_vector = vehicle_data.x;  
        y_vector = vehicle_data.y;  
          
        % 创建或选择子图  
        subplot(nRows, nCols, subplotIndex);  
        plot(x_vector, y_vector, '.');  
        
        title(sprintf('original X = %s Y = %s', num2str(x_vector(1)),num2str(y_vector(1)))); % 添加标题  
        % xlim([xmin xmax]); % xmin 和 xmax 需要定义为您想要显示的范围  
        % ylim([ymin ymax]); % ymin 和 ymax 同样需要定义  
         hold on 
        % 更新子图索引  
        
          
        % 如果subplotIndex超过了子图的总数，则退出循环  
        if subplotIndex > nRows * nCols  
            break;  
        end  
          
        % 强制更新图形窗口  
         

        % 暂停0.1秒  
        %pause(0.1);  
    end  
    hold off
      subplotIndex =subplotIndex+1;
    % 如果subplotIndex超过了子图的总数，则退出外层循环  
    if subplotIndex > nRows * nCols  
        break;  
    end  
end

%%
%提取南北直行，东西直行，和转向的轨迹

% 创建 4x3 的子图网格  
nRows = 4;  
nCols = 3;  
figure; % 创建一个新的图形窗口  
  
% 初始化子图索引  
subplotIndex = 1;  
  
for j = 1:12 % 假设您只需要处理前12个索引  
    Idx = find(idx == j); % qf_Idx 需要在此前定义，并且其长度至少为12  
    unique_vehicle_id = unique_vehicle_ids(Idx);  
    
    for i = 1:numel(unique_vehicle_id)  
        flag = find(all_start_car(:,1) == unique_vehicle_id(i));
        if isempty(flag)
            continue;
        end
        
        current_vehicle_id = unique_vehicle_id(i);  
          
        % 提取当前 vehicle_id 的所有数据  
        vehicle_indices = find(data.vehicle_id == current_vehicle_id );  
        vehicle_data = data(vehicle_indices, :);  
          
        % 提取 x 和 y 坐标以及时间（时间在这里可能不需要用于绘图）  
        x_vector = vehicle_data.x;  
        y_vector = vehicle_data.y;  
        
        % 创建或选择子图  
        subplot(nRows, nCols, subplotIndex);  
        plot(x_vector, y_vector, '.');  
        
        title(sprintf('original X = %s Y = %s', num2str(x_vector(1)),num2str(y_vector(1)))); % 添加标题  
        % xlim([xmin xmax]); % xmin 和 xmax 需要定义为您想要显示的范围  
        % ylim([ymin ymax]); % ymin 和 ymax 同样需要定义  
         hold on 
        % 更新子图索引  
        
          
        % 如果subplotIndex超过了子图的总数，则退出循环  
        if subplotIndex > nRows * nCols  
            break;  
        end  

        % 暂停0.1秒  
        %pause(0.1);  
    end  
    hold off
      subplotIndex =subplotIndex+1;
    % 如果subplotIndex超过了子图的总数，则退出外层循环  
    if subplotIndex > nRows * nCols  
        break;  
    end  
end
%% 出发时间 k_means初始聚类
figure
for j = 1:12 % 假设您只需要处理前12个索引  
    Idx = find(idx == j); % qf_Idx 需要在此前定义，并且其长度至少为12  
    unique_vehicle_id = unique_vehicle_ids(Idx);  
    
    for i = 1:numel(unique_vehicle_id)  
        flag = find(all_start_car(:,1) == unique_vehicle_id(i));
        if isempty(flag)
            continue;
        end
        
        current_vehicle_id = unique_vehicle_id(i);  

        vehicle_data = all_start_car(flag, :);    %车辆的编号和出发时刻
      
        
      
       
        % 创建或选择子图  
        if j == 1 || j== 5  %东西 直的
          
            plot(vehicle_data(3),1,'ro');  %red

            hold on
        end


        if j == 3|| j == 6 %we右
            plot(vehicle_data(3),1,'r*');  %red
           hold on
     
        end

        if j == 7|| j == 8  %sn右
            plot(vehicle_data(3),1,'g*');  %green
        hold on
        end

        if j == 11 || j == 12   %南北  直的
            plot(vehicle_data(3),1,'go');  %green
        hold on
        end

        if j == 4    %we左拐
            plot(vehicle_data(3),1,'b^');  %blue
        hold on
        end

        if j == 2|| j == 10    %sn左拐
            plot(vehicle_data(3),1,'bo');  %green
         hold on
        end
        hold on
       
    end  
    
 
end

        
hold off





%% 聚类处理后，消除噪声

 % 创建一个新的图形窗口  
  south_north = [];
  west_east = [];
  sn_left = [];
  we_left = [];
 buffer_cluster = 15;
    figure
for j = 1:12 % 假设您只需要处理前12个索引  
    Idx = find(idx == j); % qf_Idx 需要在此前定义，并且其长度至少为12  
    unique_vehicle_id = unique_vehicle_ids(Idx);  
    
    for i = 1:numel(unique_vehicle_id)  
        all_start_car1 = sortrows(all_start_car,3);
        flag = find(all_start_car1(:,1) == unique_vehicle_id(i));
        if isempty(flag)
            continue;
        end
        
        %current_vehicle_id = unique_vehicle_id(i);  
        
        vehicle_data = all_start_car1(flag, :);    %车辆的编号和出发时刻
        
        
      
       
        % 创建或选择子图  
        if j == 1 || j== 5 || j == 3|| j == 6  %东西 直的 +右拐
          if isempty(west_east)||abs(vehicle_data(3) - current_west_east) > buffer_cluster  %每类只保留的一个
                current_west_east = vehicle_data(3);
           
                west_east = [west_east;current_west_east];
                plot(current_west_east,1,'ro');  
          end
 
            hold on
        end

%{
        if j == 3|| j == 6 %we右
            %plot(vehicle_data(3),1,'r*');  %red
           hold on
              
        end
%} 
%{
        if j == 7|| j == 8  %sn右
            %plot(vehicle_data(3),1,'g*');  %green
        hold on
        end
%}
        if j == 11 || j == 12 ||j == 7|| j == 8  %南北  直的 +右转
            if isempty(south_north)||abs(vehicle_data(3) - current_south_north) > buffer_cluster  %每类只保留的一个
                current_south_north = vehicle_data(3);
           
                south_north  = [south_north;current_south_north];
                plot(current_south_north,1,'go');  %green 
            end
        hold on
        end

        if j == 4    %we左拐
            if isempty(we_left)||abs(vehicle_data(3) - current_we_left) > buffer_cluster  %每类只保留的一个
                current_we_left = vehicle_data(3);
           
                we_left  = [we_left;current_we_left];
                plot(current_we_left,1,'m*');  %green 
            end
           
        hold on
        end

        if j == 2|| j == 10    %sn左拐
            if isempty(sn_left)||abs(vehicle_data(3) - current_sn_left) > buffer_cluster  %每类只保留的一个
                current_sn_left = vehicle_data(3);
           
                sn_left = [sn_left;current_sn_left];
                plot(current_sn_left,1,'b^');  %green 
                
                hold on
            end
        end
        hold on
       
    end  
    
 
end
%legend('show')
        
hold off
south_north = clusterAndKeepMin(south_north);
  west_east = clusterAndKeepMin(west_east);
  sn_left = clusterAndKeepMin(sn_left);
  we_left = clusterAndKeepMin(we_left);
  figure
  hold on
  plot(sn_left,1,'b^', 'DisplayName','南北左转');

  plot(we_left,1,'m*', 'DisplayName', '东西左转');

  plot(south_north,1,'go', 'DisplayName', '南北直行+右转');
 
  plot(west_east,1,'ro', 'DisplayName', '东西直行+右转');  
 
  hold off
  T_circle = [];
  T_circle = [west_east(1:47),we_left(1:47),south_north(1:47),sn_left(1:47)];
  T = diff(T_circle);
  diff_T_circle = zeros(size(T_circle));
mode(T)
% 计算每一列当前列减前一列的差值
% for n = 1:size(T_circle, 2)
% 
%     diff_T_circle(:, n) = T_circle(:, n) - T_circle(:, n-1);
% end

 % T_circle = concatenateAndFilter(T_circle)
% %%
%   south_north = [];
%   west_east = [];
%   sn_left = [];
%   we_left = [];
%  buffer_cluster = 15;
% figure
% 
% for j = 1:12
%     Idx = find(idx == j);
%     unique_vehicle_id = unique_vehicle_ids(Idx);
% 
%     for i = 1:numel(unique_vehicle_id)  
%         flag = find(all_start_car(:,1) == unique_vehicle_id(i));
%         if isempty(flag)
%             continue;
%         end
% 
%         vehicle_data = all_start_car(flag, :);
% 
%         if j == 1 || j == 5 || j == 3 || j == 6
%             % 东西直行 + 右转
%             if isempty(west_east) || (vehicle_data(3) - current_west_east) > buffer_cluster
%                 current_west_east = vehicle_data(3);
%                 west_east = [west_east; current_west_east];
%                 plot(current_west_east, 1, 'ro', 'DisplayName', '东西直行+右转');
%             end
%         elseif j == 11 || j == 12 || j == 7 || j == 8
%             % 南北直行 + 右转
%             if isempty(south_north) || (vehicle_data(3) - current_south_north) > buffer_cluster
%                 current_south_north = vehicle_data(3);
%                 south_north = [south_north; current_south_north];
%                 plot(current_south_north, 1, 'go', 'DisplayName', '南北直行+右转');
%             end
%         elseif j == 4
%             % 东西左转
%             if isempty(we_left) || (vehicle_data(3) - current_we_left) > buffer_cluster
%                 current_we_left = vehicle_data(3);
%                 we_left = [we_left; current_we_left];
%                 plot(current_we_left, 1, 'm*', 'DisplayName', '东西左转');
%             end
%         elseif j == 2 || j == 10
%             % 南北左转
%             if isempty(sn_left) || (vehicle_data(3) - current_sn_left) > buffer_cluster
%                 current_sn_left = vehicle_data(3);
%                 sn_left = [sn_left; current_sn_left];
%                 plot(current_sn_left, 1, 'b^', 'DisplayName', '南北左转');
%             end
%         end
%         hold on
%     end  
% end
% hold off
%%
figure
plot(1,'ro')
hold on
plot(1,'go')
plot(1,'m*')
plot(1,'b^')
legend('东西直行+右转', '南北直行+右转', '东西左转', '南北左转');
