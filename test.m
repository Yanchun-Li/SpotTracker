clc; clear;
server_port = 5005;
tcpServer = tcpserver('0.0.0.0', server_port);
disp(['TCP server listening on port ', num2str(server_port)]);


% PID Parameter
Kp = 1;
Ki = 0.1;
Kd = 0.05;

integral_x = 0;
previous_error_x = 0;
integral_y = 0;
previous_error_y = 0;
dt = 0.1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialize the stage %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% コントローラ設定ソフト GSC-01cfg (prismにインストール済み)で電流値を750mA, 逆転, ハーフピッチに設定する
exist n
if(ans==0)
    n=1;
end
%% Setup Controller stage 
%% Axis1 OSMS26-200(X), Axis2 OSMS26-200(Y)
stg1 = instrfind('Type', 'serial', 'Port', 'COM4', 'Tag', '');      %デバイスマネージャーで確認
if isempty(stg1)
    stg1 = serial('COM4');
else
    fclose(stg1);
    stg1 = stg1(1);
end

stg2 = instrfind('Type', 'serial', 'Port', 'COM8', 'Tag', '');
if isempty(stg2)
    stg2 = serial('COM8');
else
    fclose(stg2);
    stg2 = stg2(1);
end

% Connect to instrument object, obj1.
fopen(stg1);
set(stg1, 'BaudRate', 38400);                           % 速度変えたければ 38400
set(stg1, 'FlowControl', 'hardware');
set(stg1, 'Terminator', {'CR/LF','CR/LF'});
set(stg1, 'Timeout', 5);
flushinput(stg1);                                       % flush the data buffer.
mywait(stg1)                                            % wait until the reset gets settled. %%%%

fopen(stg2);
set(stg2, 'BaudRate', 38400);
set(stg2, 'FlowControl', 'hardware');
set(stg2, 'Terminator', {'CR/LF','CR/LF'});
set(stg2, 'Timeout', 5);
flushinput(stg2);                                       % flush the data buffer.
mywait(stg2)                                            % wait until the reset gets settled. %%%%

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
    % ----------------- PID Control (Y-axis) -----------------
    integral_x = integral_x + e_x * dt;
    % derivative_x = (e_x - previous_error_x) / dt;
    P_x = Kp * e_y;
    % I_x = Ki * integral_x;
    % D_x = Kd * derivative_x;
    % output_y = P_y + I_y + D_y;
    output_x = P_x;
    % previous_error_x = e_x;
    disp(['PID Y: P_y = ', num2str(P_x),'Output = ', num2str(output_x)]);
    
    % ----------------- PID Control (Y-axis) -----------------
    integral_y = integral_y + e_y * dt;
    % derivative_y = (e_y - previous_error_y) / dt;
    P_y = Kp * e_y;
    % I_y = Ki * integral_y;
    % D_y = Kd * derivative_y;
    % output_y = P_y + I_y + D_y;
    output_y = P_y;
    % previous_error_y = e_y;
    disp(['PID Y: P_y = ', num2str(P_y),'Output = ', num2str(output_y)]);

    % ---------------- Move stage [2um/pulse] ---------------------
    xpulse = output_x * 1000 / 2;                       % x axis convert pixel to pulse [2um/pulse]
    ypulse = output_y * 1000 / 2;                       % y axis convert pixel to pulse

    fprintf(stg1, strcat('M:2+P', num2str(xpulse)));    % move x axis
    fprintf(stg2, strcat('M:1+P', num2str(ypulse)));    % move y axis
    
    fprintf(stg1, 'G:');
    mywait(stg1);
    fprintf(stg2, 'G:');
    mywait(stg2);        

    pause(0.01);
end
