clc;
clear;

% 设置 TCP 服务器，监听所有 IP 地址
server_port = 5005;  % 端口号
tcpServer = tcpserver('0.0.0.0', server_port);  % 创建 TCP 服务器

disp(['TCP 服务器已启动，正在监听端口 ', num2str(server_port), '...']);

while true
    if tcpServer.NumBytesAvailable > 0
        % 读取数据（字节流）
        data = read(tcpServer, tcpServer.NumBytesAvailable, "uint8");

        % 打印接收到的字节流
        disp('Received byte data:');
        disp(data);

        % 将字节流转换为字符串
        jsonString = char(data');

        % 打印转换后的字符串
        disp('Converted to string:');
        disp(jsonString);

        % 使用 jsondecode 将字符串转换为 MATLAB 数组或元组
        try
            parsedData = jsondecode(jsonString);
            disp('Parsed JSON:');
            disp(parsedData);  % 应该是一个 MATLAB 数组，类似于 [323, 1]
        catch ME
            disp('Error decoding JSON:');
            disp(ME.message);  % 打印错误信息
        end
    end
    pause(0.1);  % 防止 CPU 占用过高
end

% 如果不再需要，关闭 TCP 服务器
clear tcpServer;
