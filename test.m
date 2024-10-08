clc;
clear;

% 设置 TCP 服务器，监听所有 IP 地址
server_port = 5005;     % 端口号
tcpServer = tcpserver('0.0.0.0', server_port);  % 创建 TCP 服务器

disp(['TCP 服务器已启动，正在监听端口 ', num2str(server_port), '...']);

% 循环监听，接收数据
while true
    % 检查是否有数据可读取
    if tcpServer.NumBytesAvailable > 0
        data = read(tcpServer, tcpServer.NumBytesAvailable, "uint8");  % 读取数据
        disp(['接收到的数据: ', char(data')]);                        % 将字节数据转换为字符并显示
    end
    pause(0.1);  % 防止循环占用过高 CPU
end

% 如果不再需要，关闭 TCP 服务器
clear tcpServer;
