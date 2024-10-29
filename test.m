clc; clear;


%%%%%%%%%%%%%%% Initialize the stage %%%%%%%%%%%%%%
exist n
if(ans==0)
    n=1;
end
%% Setup Controller stage 
%% Axis1 OSMS26-200(X)
stg1 = instrfind('Type', 'serial', 'Port', 'COM4', 'Tag', '');      %デバイスマネージャーで確認
if isempty(stg1)
    stg1 = serial('COM4');
else
    fclose(stg1);
    stg1 = stg1(1);
end
%  Axis2 OSMS26-200(Y)
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


server_port = 5005;
tcpServer = tcpserver('0.0.0.0', server_port); 
disp(['TCP server listening on port ', num2str(server_port)]);

L = 5;              % distance from mirror to target
alpha = 0.01;
beta = 0.03;

xs = 0; ys = 0;
phi = 0; theta = 0;

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
        
            disp(['Received error: ', num2str(e), 'ex, ey = ', num2str(ex, ey)]);

            xs = xs - alpha * (ex / e);
            ys = ys - alpha * (ey / e);

            phi = phi - beta * (L / cos(phi)^2);
            theta = theta + beta * (L / cos(theta)^2);

            fprintf('Stage Position: (xs: %f, ys: %f)\n', xs, ys);
            fprintf('Mirror Angles: (phi: %f, theta: %f)\n', phi, theta);

        catch e
            disp('Error processing data:');
            disp(e.message);
        end
    end
end