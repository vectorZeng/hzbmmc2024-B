% 异常值检验
clear;clc
data = readmatrix("A1.csv");   
  
% 获取唯一的 vehicle_id  
unique_vehicle_ids = unique(data(:, 2));  
% 初始化一个逻辑数组来标记异常值  
is_anomaly = false(size(unique_vehicle_ids, 1), 1); 
  
% 遍历每个唯一的 vehicle_id  
for i = 1:length(unique_vehicle_ids)  
    current_vehicle_id = unique_vehicle_ids(i);  
      
    % 提取当前 vehicle_id 的数据子集  
    current_data = data(data(:, 2) == current_vehicle_id, :);  
      
    % 检查 x 坐标是否随着 time 的增加而单调递增  
    if ~all(diff(current_data(:, 3)) >= 0)  
        % 找出不满足单调性的行索引  
        idx = find(diff(current_data(:, 3)) < 0) + 1;  
          
        % 将这些行索引转换为原始数据中的行索引  
        original_idx = find(data(:, 2) == current_vehicle_id & ...  
                            ismember(data(:, 1), current_data(idx, 1)));  
          
        % 标记这些行为异常值  
        is_anomaly(i) = true;  
    end  
end  