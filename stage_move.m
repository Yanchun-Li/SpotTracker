clc
%clear
close all
delete(visadevfind)

% PID Parameter
Kp = 1;
Ki = 0.1;
Kd = 0.05;

integral_x = 0;
previous_error_x = 0;
integral_y = 0;
previous_error_y = 0;
dt = 0.1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Connect Windows PC %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t = tcpip('0.0.0.0', 5005, 'NetworkRole', 'server');    % Create a server for receiving data
fopen(t);                                               % open the connection

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialize the Lock-in Amplifier %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Preparation for Communication with Oscilloscope
visa_brand = 'ni';
visa_address = 'USB0::0x0699::0x03C7::C023076::INSTR';
osc = visadev(visa_address);
fopen(osc);                                             % Open instrument with a default buffer size
record = str2double(query(osc, 'hor:reco?'));           % Get the current record length
fclose(osc);                                            % Close the instrument to set the buffer size
% Calculate required buffer size and set it
required_buffer = record * 2;                           % Adjust as needed
osc.InputBufferSize = required_buffer;
osc.OutputBufferSize = required_buffer;

% Reopen the instrument
fopen(osc);
fwrite(osc, 'data:source CH1');

% Set Lockin amplifier and Function generator
amp = visadev('USB0::0x0D4A::0x0049::9382688::INSTR');  %LI5660
fg = visadev('USB0::0x0D4A::0x000D::9244234::INSTR');   %NF1973
fopen(amp);
fopen(fg);

% ファイル名を設定
current_time = datetime('now', 'Format', 'yyyyMMdd_HHmmss');
csv_filename = sprintf('./data_all/%s.csv', current_time);
% Write the header to the file
writematrix(["Time", "Lockin Signal"], csv_filename);

duty=50;
fprintf(fg, [':Source1:FUNCtion:SQUare:DCYCle ', num2str(duty)]);
fprintf(fg, ':Source1:VOLT 4.9');
fprintf(fg, ':Source1:VOLT:OFFS 2.5')
fprintf(fg, ':CHANnel:MODE PHASe' );
fprintf(fg, ':OUTPut1:STATe OFF' );

%%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialize the Mirror %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Set Python
pyenv("Version", "C:\Users\Lenovo\AppData\Local\Programs\Python\Python310\python.exe");
optoMDC = py.importlib.import_module('optoMDC');
% Connect to Mirror
port = "COM4";
mre2 = optoMDC.connect(port=port);
% Set x and y to closed loop for control
ch_x = mre2.Mirror.Channel_0;
ch_x.SetControlMode(optoMDC.Units.XY);
static_input_x = ch_x.StaticInput;
ch_y = mre2.Mirror.Channel_1;
ch_y.SetControlMode(optoMDC.Units.XY);
static_input_y = ch_y.StaticInput;
% Hill Climbing
% Initial position
X = 0;
Y = 0;
xValue = clamp(angle_to_xy(X), -1, 1);
static_input_x.SetXY(xValue);
yValue = clamp(angle_to_xy(Y), -1, 1);
static_input_y.SetXY(yValue);

threshold = 0.6;

xvalueList = static_input_x.GetXY();
yvalueList = static_input_y.GetXY();

% Step size in degree
step = 1;

volt_history = [];
runtime= [];

% Initialize the current voltage
currentX = xvalueList{1};
currentY = yvalueList{1};
current_time = cputime;
runtime = [runtime, cputime - current_time];
[currentVal, volt_history] = getScopeData(osc, volt_history, runtime);
step = setStep(currentVal, step);
fprintf('The current position is:(%f, %f, %f).\n', currentX, currentY, currentVal);


while true
    data = fscanf(t);                   % read data from Raspi as JSON (error_x, error_y)
    error_data = jsondecode(data);      % decode JSON data
    e_x = error_data(1);
    e_y = error_data(2);
    
    disp(['Received error: e_x = ', num2str(e_x), ', e_y = ', num2str(e_y)]);
    
    % ----------------- PID Control (Y-axis) -----------------
    integral_x = integral_x + e_x * dt;
    derivative_x = (e_x - previous_error_x) / dt;
    P_x = Kp * e_y;
    I_x = Ki * integral_x;
    D_x = Kd * derivative_x;
    % output_y = P_y + I_y + D_y;
    ouput_x = P_x
    previous_error_x = e_x;
    disp(['PID Y: P_y = ', num2str(P_x),'Output = ', num2str(output_x)]);
    
    % ----------------- PID Control (Y-axis) -----------------
    integral_y = integral_y + e_y * dt;
    derivative_y = (e_y - previous_error_y) / dt;
    P_y = Kp * e_y;
    I_y = Ki * integral_y;
    D_y = Kd * derivative_y;
    % output_y = P_y + I_y + D_y;
    output_y = P_y;
    previous_error_y = e_y;
    disp(['PID Y: P_y = ', num2str(P_x),'Output = ', num2str(output_x)]);

    % ---------------- Move stage [2um/pulse] ---------------------
    xpulse = output_x * 1000 / 2;                       % x axis convert pixel to pulse [2um/pulse]
    ypulse = output_y * 1000 / 2;                       % y axis convert pixel to pulse
    
    % xpulse = PIDmovestage(e_x, previous_error_x)
    % ypulse = PIDmovestage(e_y, previous_error_y)

    fprintf(stg1, strcat('M:2+P', num2str(xpulse)));    % move x axis
    fprintf(stg2, strcat('M:1+P', num2str(ypulse)));    % move y axis
    
    fprintf(stg1, 'G:');
    mywait(stg1);
    fprintf(stg2, 'G:');
    mywait(stg2);

    [currentX, currentY, currentVal, step] = adjustMirrorHorizontal(currentX, currentY, step, osc, threshold, static_input_x);
    [currentX, currentY, currentVal, step] = adjustMirrorVertical(currentX, currentY, step, osc, threshold, static_input_y);
    fprintf('Updated position: X = %f, Y = %f, Voltage = %f\n', currentX, currentY, currentVal);
    
    if currentVal > threshold
        performLockinMeasurement(osc, amp, fg, csv_filename);
        break;
    end
    
    pause(0.1);                                         
end

figure
figureHandle = figure(1);                                                % choose to save which figure
saveResultsAndDisconnect(figureHandle, osc, amp, fg, mre2, comment);    % save results and disconnect osc, amp, fg

fclose(t);
delete(t);

fclose(stg1);
fclose(stg2);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Functions Definition %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function mywait(obj)
    flushinput(obj);
    fprintf(obj, '!:');
    status=fgetl(obj);
    while(status=='B')
        pause(0.1)
        fprintf(obj, '!:');
        status=fgetl(obj);
    end
end

function [bestX, bestY] = grid_search(initialX, initialY, stepSize, searchRangeX, searchRangeY, osc)
    bestX = initialX;
    bestY = initialY;
    bestValue = -Inf; 

    for x = initialX-searchRangeX:stepSize:initialX+searchRangeX
        for y = initialY-searchRangeY:stepSize:initialY+searchRangeY
            set_mirror_position(x, y);
            currentValue = getScopeData(osc);
            if currentValue > bestValue
                bestX = x;
                bestY = y;
                bestValue = currentValue;
            end    
            fprintf('Grid Search: X = %f, Y = %f, Value = %f\n', x, y, currentValue);
        end
    end
end

function [optimalX, optimalY] = gradient_descent(initialX, initialY, osc, maxIterations, lr)
    X = initialX;
    Y = initialY;
    prevValue = getScopeData(osc);              % get previous value
    
    for i = 1:maxIterations
        deltaX = 0.001;
        deltaY = 0.001;
        
        % gradient in x-axis
        set_mirror_position(X + deltaX, Y);
        valueX = getScopeData(osc);
        gradX = (valueX - prevValue) / deltaX;
        
        % gradient in y-axis
        set_mirror_position(X, Y + deltaY);
        valueY = getScopeData(osc);
        gradY = (valueY - prevValue) / deltaY;

        X = X + lr * gradX;
        Y = Y + lr * gradY;
        prevValue = getScopeData(osc);
        fprintf('Iteration %d: X = %f, Y = %f, Value = %f\n', i, X, Y, prevValue);

        if abs(gradX) < 1e-4 && abs(gradY) < 1e-4
            break;
        end
    end
    
    optimalX = X;
    optimalY = Y;
end

function pulse, previous_e = PIDmovestage(e, previous_e, integral)
    % ----------------- PID Control (X-axis) -----------------
    integral_x = integral + e * dt;
    derivative_x = (e - previous_e) / dt;
    P_x = Kp * e_x;
    I_x = Ki * integral_x;
    D_x = Kd * derivative_x;
    % output_x = P_x + I_x + D_x; 
    output_x = P_x
    previous_e = e_x; 
    pulse = output_x * 1000 / 2;                       % x axis convert pixel to pulse [2um/pulse]
    disp(['PID X: P_x = ', num2str(P_x),'Output = ', num2str(output_x)]);
end

function xy_value = angle_to_xy(angle)
    xy_value = tan(deg2rad(angle)) / tan(deg2rad(50));
end

%%

function xy_value = clamp(value, min_value, max_value)
    xy_value = max(min_value, min(value, max_value));
end

%%
function step = setStep(currentVoltage, step)
    if currentVoltage < 0.1
        step = 1*sign(step); 
    elseif currentVoltage < 0.2
        step = 0.5*sign(step);
    elseif currentVoltage < 0.3
        step = 0.1*sign(step);
    elseif currentVoltage < 0.35
        step = 0.05*sign(step);
    else
        step = 0.01*sign(step);
    end
end

function [currentX, currentY, currentVal, step] = adjustMirrorHorizontal(currentX, currentY, step, osc, threshold, static_input_x)
    horizental_direction = 0;
    current_time = cputime;
    [currentVal, volt_history] = getScopeData(osc, [], [cputime - current_time]);
    
    while currentVal < threshold
        X = currentX + step;
        Y = currentY;
        xValue = clamp(angle_to_xy(X), -1, 1);
        static_input_x.SetXY(xValue); % set mirror
        xvalueList = static_input_x.GetXY();
        
        % Wait until the mirror reaches the destination position
        while abs(xvalueList{1} - xValue) >= 0.001
            xvalueList = static_input_x.GetXY();
        end
        
        [nextVal, ~] = getScopeData(osc, [], [cputime - current_time]);
        fprintf('The current position is:(%f, %f, %f).\n', X, Y, nextVal);
        
        if nextVal - currentVal > 0
            nextStep = setStep(nextVal, step);
        else
            X = currentX;
            xValue = clamp(angle_to_xy(X), -1, 1);
            static_input_x.SetXY(xValue); % set mirror
            while abs(xvalueList{1} - xValue) >= 0.001
                xvalueList = static_input_x.GetXY();
            end
            [nextVal, ~] = getScopeData(osc, [], [cputime - current_time]);
            fprintf('The current position is:(%f, %f, %f).\n', X, Y, nextVal);
            nextStep = setStep(nextVal, step);
            nextStep = -nextStep;
            horizental_direction = horizental_direction + 1;
        end
        
        % Move to the next position
        currentX = X;
        currentY = Y;
        currentVal = nextVal;
        step = nextStep;
        
        if horizental_direction >= 2
            break;
        end
    end
end

function [currentX, currentY, currentVal, step] = adjustMirrorVertical(currentX, currentY, step, osc, threshold, static_input_y)
    vertical_direction = 0;
    current_time = cputime;
    [currentVal, volt_history] = getScopeData(osc, [], [cputime - current_time]);
    
    while currentVal < threshold
        X = currentX;
        Y = currentY + step;
        yValue = clamp(angle_to_xy(Y), -1, 1);
        static_input_y.SetXY(yValue); % set mirror
        yvalueList = static_input_y.GetXY();
        
        % Wait until the mirror reaches the destination position
        while abs(yvalueList{1} - yValue) >= 0.001
            yvalueList = static_input_y.GetXY();
        end
        
        [nextVal, ~] = getScopeData(osc, [], [cputime - current_time]);
        fprintf('The current position is:(%f, %f, %f).\n', X, Y, nextVal);
        
        if nextVal - currentVal > 0
            nextStep = setStep(nextVal, step);
        else
            Y = currentY;
            yValue = clamp(angle_to_xy(Y), -1, 1);
            static_input_y.SetXY(yValue); % set mirror
            while abs(yvalueList{1} - yValue) >= 0.001
                yvalueList = static_input_y.GetXY();
            end
            [nextVal, ~] = getScopeData(osc, [], [cputime - current_time]);
            fprintf('The current position is:(%f, %f, %f).\n', X, Y, nextVal);
            nextStep = setStep(nextVal, step);
            nextStep = -nextStep;
            vertical_direction = vertical_direction + 1;
        end
        
        % Move to the next position
        currentX = X;
        currentY = Y;
        currentVal = nextVal;
        step = nextStep;
        
        if vertical_direction >= 2
            break;
        end
    end
end


function performLockinMeasurement(osc, amp, fg, csv_filename)
    n = 256;
    freq = 64e3;
    label_time = [];
    label_freq = [];
    lockin_signal = zeros(1, n);
    lockin_signal_amp = zeros(1, n);
    lockin_signal_phase = zeros(1, n);
    
    tic;
    for freq_cnt = 1:n
        freq = freq + 0.5e3;
        [label_freq, lockin_signal, current_data] = lockinMeasurement(freq, freq_cnt, ...
            toc, label_freq, lockin_signal, lockin_signal_amp, lockin_signal_phase, ...
            amp, fg);
        
        writematrix(current_data, csv_filename, 'WriteMode', 'append');
    end
end

function saveResultsAndDisconnect(figureHandle, osc, amp, fg, mre2, comment)
    fprintf('Saving and cleaning up...\n');
    path = './data_all/';
    if ~exist(path, 'dir')
        mkdir(path);
    end
    
    datetime.setDefaultFormats('default', 'yyyy-MM-dd-HHmmss');
    dt = datetime('now');
    time = string(dt);
    extension = '.mat';
    comment = ['comment']
    filename = append(path, time, comment);
    

    save(strcat(filename, extension));
    extension = '.fig';
    saveas(figureHandle, strcat(filename, '_spectrum', extension));
    
    % disconnect
    mre2.disconnect();
    fclose(osc);
    fclose(amp);
    fclose(fg);
    
    fprintf('Results saved successfully.\n');
end