clc; clear;
server_port = 5005;
tcpServer = tcpserver('0.0.0.0', server_port);
disp(['TCP server listening on port ', num2str(server_port)]);

while true
    if tcpServer.NumBytesAvailable > 0
        try
            % 读取原始字节数据
            rawData = read(tcpServer, tcpServer.NumBytesAvailable, "uint8");
            
            % 尝试不同的编码方式转换为字符串
            dataStr = native2unicode(rawData, 'UTF-8');
            if isempty(dataStr)
                dataStr = native2unicode(rawData, 'ASCII');
            end
            
            % 显示原始数据
            % disp('Received raw data:');
            % disp(rawData);
            
            % 显示转换后的字符串
            disp('Received data as string:');
            disp(dataStr);
            
            % 尝试解析JSON
            % parsedData = jsondecode(dataStr);
            % disp('Parsed JSON data:');
            % disp(parsedData);
        catch e
            disp('Error processing data:');
            disp(e.message);
        end
    end
    pause(0.01);
end