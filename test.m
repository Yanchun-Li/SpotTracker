clc;
clear;

% 设置 TCP 服务器，监听所有 IP 地址
server_port = 5005;  % 端口号
tcpServer = tcpserver('0.0.0.0', server_port);  % 创建 TCP 服务器

disp(['TCP 服务器已启动，正在监听端口 ', num2str(server_port), '...']);

buffer = "";  % 用于存储拼接后的字符串

while true
    if tcpServer.NumBytesAvailable > 0
        % 读取数据（字节流）
        data = read(tcpServer, tcpServer.NumBytesAvailable, "uint8");

        % 将字节流转换为字符串，确保编码正确
        jsonString = native2unicode(data', 'UTF-8');  % 将字节流转换为 UTF-8 字符串
        buffer = strcat(buffer, jsonString);  % 拼接到缓冲区

        % 使用 strjoin 将字符数组转换为单个字符串
        bufferJoined = strjoin(cellstr(buffer), '');

        % 打印拼接后的字符串
        disp('Received and concatenated string:');
        disp(bufferJoined);

        % 检查是否接收到完整的 JSON（假设为数组，检测 []）
        if startsWith(bufferJoined, '[') && endsWith(bufferJoined, ']')
            try
                % 使用 jsondecode 将拼接的完整字符串转换为 MATLAB 数组
                parsedData = jsondecode(bufferJoined);  % 解析 JSON 数据
                disp('Parsed JSON:');
                disp(parsedData);  % 打印解析后的 MATLAB 数组，例如 [330, 10]
                buffer = "";  % 清空缓冲区，准备接收下一条数据
            catch ME
                disp('Error decoding JSON:');
                disp(ME.message);  % 打印错误信息，继续接收数据
            end
        end
    end
    pause(0.1);  % 防止 CPU 占用过高
end

% 如果不再需要，关闭 TCP 服务器
clear tcpServer;
