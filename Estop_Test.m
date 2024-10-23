% Set up the serial communication
try
    serialObj = serialport('COM3', 9600); % Replace 'COM3' with the correct COM port
catch
    disp('Error opening serial port. Check the connection and try again.');
    return;  % Exit the script if there's an error
end

configureTerminator(serialObj, "LF"); % Line feed as the terminator

r = UR3;
q1 = [0,0,0,0,0,0];
q2 = [2*pi,0,0,0,0,0];
steps = 200;
qMatrix = jtraj(q1, q2, steps);

disp('Robot is running. Press the E-Stop to pause.');

data = '';  

% Animation loop
for i = 1:steps
    % Check if the E-Stop button is pressed
    if serialObj.NumBytesAvailable > 0
        data = readline(serialObj);  % Read the data from STM32

        if strcmp(data, 'PAUSE')      % If STM32 sends 'PAUSE' command
            disp('E-Stop button pressed. Pausing the robot.');
            pause;  % Pause execution until a key is pressed
        end
    end
 
    r.model.animate(qMatrix(i, :));
    drawnow(); 
    pause(0.1)  % Adjust the pause duration as needed
end

% Clean up the serial object
% clear serialObj;

disp('Program terminated.');

% Now data will be available in the workspace
disp(['Last received data: ', data]);
