clc; clear;
server_port = 5005;
tcpServer = tcpserver('192.168.100.29', server_port);
disp(['TCP server listening on port ', num2str(server_port)]);

while true
    if tcpServer.NumBytesAvailable > 0
        try
            rawData = read(tcpServer, tcpServer.NumBytesAvailable, "uint8");
            dataStr = native2unicode(rawData, 'UTF-8');
            if isempty(dataStr)
                dataStr = native2unicode(rawData, 'ASCII');
            end
            % disp('Received data as string:');
            % disp(dataStr);
            dataStr = strrep(dataStr, '[', '');
            dataStr = strrep(dataStr, ']', '');
            data_parts = strsplit(dataStr, ',');
            ex = str2double(data_parts{1}); 
            ey = str2double(data_parts{2}); 
            disp("Error Data:", ex, ey)
        catch e
            disp('Error processing data:');
            disp(e.message);
        end
    end
end
