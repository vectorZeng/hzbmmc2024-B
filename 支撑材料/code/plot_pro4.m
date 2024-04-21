clear;clc
data = readmatrix("D.csv");

% 提取唯一的vehicle_id
unique_vehicle_ids = unique(data(:, 2));
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
    
end