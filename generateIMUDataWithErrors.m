function dataWithErrors = generateIMUDataWithErrors(originalIMUData, R, eb, db, web, wdb)
    % 初始化输出数据集合
    [n, m] = size(originalIMUData);
    dataWithErrors = zeros(n, m, R); % 预分配空间以存储R组数据

    for i = 1:R
        % 使用imuerrset设置误差参数
        imuerr = imuerrset(eb, db, web, wdb);
        
        % 使用imuadderr函数添加误差
        imuDataWithError = imuadderr(originalIMUData, imuerr);
        
        % 将带有误差的数据存储到输出数据集合中
        dataWithErrors(:,:,i) = imuDataWithError;
    end
end
