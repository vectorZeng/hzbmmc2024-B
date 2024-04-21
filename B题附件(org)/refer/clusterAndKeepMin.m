function clustered_data = clusterAndKeepMin(data)
       % 初始化一个空数组来存放聚类后的数据
    clustered_data = [];

    % 对数据进行排序
    sorted_data = sort(data);

    % 遍历排序后的数据
    for i = 1:length(sorted_data)
        % 判断当前数据与已聚类数据中的最后一个数据的差是否小于15
        if isempty(clustered_data) || abs(sorted_data(i) - clustered_data(end)) >= 25
            % 如果差大于等于15，则将当前数据添加到聚类后的数据中
            clustered_data = [clustered_data,;sorted_data(i)];
        end
    end
end