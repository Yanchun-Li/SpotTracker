clc; clear;


% ------------------ Setup Controller Stage ------------------
stg1 = setup_serial_stage('COM4');
stg2 = setup_serial_stage('COM8');

% --------------- Configure Python environment----------------
python_version = "C:\Users\Lenovo\AppData\Local\Programs\Python\Python310\python.exe";
configure_python_env(python_version);  % Set Python environment
port = "COM4";    % COM port, ensure the correct port is used

% Connect to the mirror device (initialize once)
try
    optoMDC = py.importlib.import_module('optoMDC');
    mre2 = optoMDC.connect(port=port);
    disp(['Successfully connected to port: ', port]);
catch e
    disp(e.message);
    return;
end

% ---------------- Connect tcp/ip port ------------------
server_port = 5005;
tcpServer = tcpserver('0.0.0.0', server_port); 
disp(['TCP server listening on port ', num2str(server_port)]);

L = 5;              % distance from mirror to target[cm]
alpha = 0.01;
beta = 0.03;

error_thres = 100;
error_log = [];
xs = 0; ys = 0;
phi = 0; theta = 0;

%------------------------ Gradient Descent - Stage & Mirror ------------------------
while true
    if tcpServer.NumBytesAvailable > 0
        try
            dataStr = readline(tcpServer);
            % disp('Received data as string:');
            % disp(dataStr);
            error_data = jsondecode(dataStr);
            % disp('Decoded Error Data:');
            % disp(error_data);
            ex = error_data(1);
            ey = error_data(2);
            e = sqrt(ex^2 + ey^2);
            error_log = [error_log; e]; 

            disp(['Received error: ', num2str(e), 'ex, ey = ', num2str(ex, ey)]);
            % Received error: 384.6414   ex, ey = -382-45

            if e < error_thres
                disp(['Error is below threshold: ', num2str(e), 'Converting to HillClimb ...']);
                break;
            end
        
            xs = xs - alpha * (ex / e);
            ys = ys - alpha * (ey / e);
            phi = phi - beta * (L / cos(phi)^2);
            theta = theta + beta * (L / cos(theta)^2);

            rotate_mirror_at_angles(mre2, phi, theta)

            fprintf('Stage Position: (xs: %f, ys: %f)\n', xs, ys);
            fprintf('Mirror Angles: (phi: %f, theta: %f)\n', phi, theta);
            
            % convert pixel to pulse (0.5mm/pixel -> 500pulse/mm)
            xpulse = round(xs * 1/2 * 1000 / 2);
            ypulse = round(ys * 1/2 * 1000 / 2);  
            % move stages           
            fprintf(stg1, strcat('M:2+P', num2str(xpulse)));  
            fprintf(stg2, strcat('M:1+P', num2str(ypulse))); 
            fprintf(stg1, 'G:');
            mywait(stg1);
            fprintf(stg2, 'G:');
            mywait(stg2);

        catch e
            disp('Error processing data:');
            disp(e.message);
        end
    end
end

% ---------------------------- Hill Climbing Loop ----------------
voltage_log = hill_climb(mre2, phi, theta);

% 保存误差和电压日志为 CSV 文件
writematrix(error_log, 'error_log.csv');
writematrix(voltage_log, 'voltage_log.csv');

disp('Error and voltage logs saved. Process complete.');


% Return the mirror to the initial position
ch_0.StaticInput.SetXY(0);
ch_1.StaticInput.SetXY(0);
disp('Mirror returned to initial position.');

mre2.disconnect();
fclose(stg1);
fclose(stg2);


% ------------------ Function: Setup Serial Stage ------------------
function stg = setup_serial_stage(port)
    stg = instrfind('Type', 'serial', 'Port', port, 'Tag', '');
    if isempty(stg)
        stg = serial(port);
    else
        fclose(stg);
        stg = stg(1);
    end

    fopen(stg);
    set(stg, 'BaudRate', 38400);
    set(stg, 'FlowControl', 'hardware');
    set(stg, 'Terminator', {'CR/LF', 'CR/LF'});
    set(stg, 'Timeout', 5);
    flushinput(stg); 
    mywait(stg);      
end

%--------------- Configure Python environment function -----------------
function configure_python_env(python_version)
    try
        pyenv("Version", python_version);
        disp("Python environment successfully configured!");
    catch e
        disp("Failed to configure Python environment. Please check the path.");
        disp(e.message);
    end
end

%--------- Core function to rotate mirror to specified angles --------
function rotate_mirror_at_angles(mre2, angle_x, angle_y)
    try
        % Calculate X and Y values for the given angles
        xy_value_x = angle_to_xy(angle_x);
        xy_value_y = angle_to_xy(angle_y);

        % Set static input for X and Y channels
        ch_0 = mre2.Mirror.Channel_0;
        ch_0.StaticInput.SetXY(xy_value_x);

        ch_1 = mre2.Mirror.Channel_1;
        ch_1.StaticInput.SetXY(xy_value_y);

        % Print the set angles and XY values
        fprintf('Set angles: X-axis = %f°, Y-axis = %f°\n', angle_x, angle_y);
        fprintf('XY values: X = %f, Y = %f\n', xy_value_x, xy_value_y);
        
        pause(0.1);
        
    catch e
        disp('Error while operating the mirror device:');
        disp(e.message);
    end
end

%--------------- Helper function to convert angles to XY coordinates--------
function xy = angle_to_xy(angle)
    try
        % Convert angle to XY coordinates
        xy = tand(angle) / tand(50);
    catch e
        disp('Error while converting angle to XY coordinates:');
        disp(e.message);
        xy = 0;
    end
end

function voltage_log = hill_climb(mre2, initial_phi, initial_theta)
    disp('Starting Hill Climbing to find the optimal mirror angles...');

    phi = initial_phi;
    theta = initial_theta;
    best_voltage = read_voltage();
    voltage_log = best_voltage;  % 保存电压日志
    step_size = 0.1;  % 每次调整角度的步长
    voltage_thres = 0.01;  % 电压变化阈值

    while true
        % 测试四个方向的角度变化
        candidates = [
            phi + step_size, theta;
            phi - step_size, theta;
            phi, theta + step_size;
            phi, theta - step_size
        ];

        found_better = false;

        % 遍历候选角度，寻找更大的电压
        for i = 1:size(candidates, 1)
            new_phi = candidates(i, 1);
            new_theta = candidates(i, 2);

            rotate_mirror_at_angles(mre2, new_phi, new_theta);
            pause(0.1);  % 稳定时间

            voltage = read_voltage();  % 获取新角度的电压
            voltage_log = [voltage_log; voltage];  % 保存电压日志

            fprintf('Tested Angles: (phi: %f, theta: %f) -> Voltage: %f\n', new_phi, new_theta, voltage);

            % 如果找到更大的电压，更新当前角度和电压
            if voltage > best_voltage + voltage_thres
                phi = new_phi;
                theta = new_theta;
                best_voltage = voltage;
                found_better = true;
                disp('Found better voltage, updating angles.');
                break;  % 跳出循环，进行下一轮测试
            end
        end

        % 如果没有找到更好的角度，停止 Hill Climbing
        if ~found_better
            disp('No better voltage found, stopping Hill Climb.');
            break;
        end
    end
end

% ---------------- Function: Read Voltage ----------------
function voltage = read_voltage()

end