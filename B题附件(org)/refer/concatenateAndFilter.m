function filtered_T_circle = concatenateAndFilter(west_east, we_left, south_north, sn_left)
    % 将四个数组串联成一个矩阵T_circle
    T_circle = [west_east, we_left, south_north, sn_left];

    % 创建一个空矩阵用于存储符合条件的行
    filtered_T_circle = [];

    % 遍历T_circle的每一行
    for i = 1:size(T_circle, 1)
        row = T_circle(i, :);

        % 计算该行中任意两个数据之间的差值
        pairwise_diff = abs(diff(nchoosek(row, 2), 1, 2));

        % 如果存在两个数据相差小于15，则舍弃该行；否则保留该行
        if any(pairwise_diff < 15)
            continue; % 舍弃该行
        else
            filtered_T_circle = [filtered_T_circle; row]; % 保留该行
        end
    end

    disp('过滤前的T_circle:');
    disp(T_circle);

    disp('过滤后的T_circle:');
    disp(filtered_T_circle);
end
