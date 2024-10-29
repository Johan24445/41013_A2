function antialarmsnoozer_gui() 
    % Create main figure for the GUI
    f = figure('Name', 'Robot Control Interface', 'NumberTitle', 'off', ...
        'Position', [100, 100, 800, 500]);  % Overall window size
    
    % Panel for UR3 Controls - positioned closer to the left and lowered
    uipanelUR3 = uipanel('Title', 'UR3 Controls', 'FontSize', 8.4, ...
        'Position', [0.03, 0.25, 0.3, 0.5]);  % Adjusted position for left margin and lowered

    % Panel for KinovaLink6 Controls - positioned closer to the right and lowered
    uipanelKinova = uipanel('Title', 'KinovaLink6 Controls', 'FontSize', 8.4, ...
        'Position', [0.67, 0.25, 0.3, 0.5]);  % Adjusted position for right margin and lowered

    % Panel for Animation Controls - moved down and narrower (50% size and reduced distance)
    uipanelAnim = uipanel('Title', 'Animation Controls', 'FontSize', 8.4, ...
        'Position', [0.05, 0.01, 0.45, 0.15]);  % Reduced width and height

    % UR3 Joint Controls
    ur3_limits = [-360 360; -360 360; -360 360; -360 360; -360 360; -360 360];

    % KinovaLink6 Joint Controls
    kinova_limits = [-360 360; -90 90; -170 170; -360 360; -360 360; -360 360];

    % Create sliders and labels for UR3 - adjusted spacing
    sliderUR3 = zeros(6, 1);
    for i = 1:6
        % Label
        uicontrol('Parent', uipanelUR3, 'Style', 'text', ...
            'String', sprintf('Joint %d', i), 'FontSize', 7, ...
            'Position', [10, 220 - (i-1)*30, 50, 15]);  % Adjusted vertical spacing

        % Value display
        uicontrol('Parent', uipanelUR3, 'Style', 'text', ...
            'String', '0°', 'FontSize', 7, ...
            'Position', [200, 220 - (i-1)*30, 50, 15], ...
            'Tag', sprintf('UR3_value_%d', i));

        % Slider
        sliderUR3(i) = uicontrol('Parent', uipanelUR3, 'Style', 'slider', ...
            'Min', ur3_limits(i, 1), 'Max', ur3_limits(i, 2), ...
            'Value', 0, 'Position', [65, 220 - (i-1)*30, 130, 14], ...
            'Tag', sprintf('UR3_slider_%d', i));
    end

    % Create sliders and labels for KinovaLink6 - adjusted spacing
    sliderKinova = zeros(6, 1);
    for i = 1:6
        % Label
        uicontrol('Parent', uipanelKinova, 'Style', 'text', ...
            'String', sprintf('Joint %d', i), 'FontSize', 7, ...
            'Position', [10, 220 - (i-1)*30, 50, 15]);  % Adjusted vertical spacing

        % Value display
        uicontrol('Parent', uipanelKinova, 'Style', 'text', ...
            'String', '0°', 'FontSize', 7, ...
            'Position', [200, 220 - (i-1)*30, 50, 15], ...
            'Tag', sprintf('Kinova_value_%d', i));

        % Slider
        sliderKinova(i) = uicontrol('Parent', uipanelKinova, 'Style', 'slider', ...
            'Min', kinova_limits(i, 1), 'Max', kinova_limits(i, 2), ...
            'Value', 0, 'Position', [65, 220 - (i-1)*30, 130, 14], ...
            'Tag', sprintf('Kinova_slider_%d', i));
    end

    % Status Text - adjusted position
    statusText = uicontrol('Parent', uipanelAnim, 'Style', 'text', ...
        'String', 'Click Start to begin simulation', 'FontSize', 7.7, ...
        'Position', [20, 80, 200, 22], ...
        'Tag', 'StatusText');

    % Start Button - moved closer to bottom
    startButton = uicontrol('Parent', uipanelAnim, 'Style', 'pushbutton', ...
        'String', 'Start', 'FontSize', 9.8, ...
        'Position', [140, 20, 100, 35], ...
        'Callback', @startSimulation);

    % Light Curtain Button - added next to the Start button
    lightCurtainButton = uicontrol('Parent', uipanelAnim, 'Style', 'pushbutton', ...
        'String', 'Light Curtain', 'FontSize', 9.8, ...
        'Position', [250, 20, 100, 35], ... % Positioned to the right of the Start button
        'Callback', @launchLightCurtain);

    % E-Stop Button - moved closer to bottom
    estopButton = uicontrol('Parent', uipanelAnim, 'Style', 'pushbutton', ...
        'String', 'E-Stop', 'FontSize', 9.8, 'BackgroundColor', 'red', ...
        'Position', [20, 20, 100, 35], ...
        'Enable', 'off', ...
        'Tag', 'EStopButton');

    % Reset UR3 button - moved up closer to sliders
    uicontrol('Parent', uipanelUR3, 'Style', 'pushbutton', ...
        'String', 'Reset UR3', 'FontSize', 7, ...
        'Position', [65, 40, 100, 20], ...
        'Tag', 'ResetUR3Button', ...
        'Enable', 'off');

    % Reset Kinova button - moved up closer to sliders
    uicontrol('Parent', uipanelKinova, 'Style', 'pushbutton', ...
        'String', 'Reset Kinova', 'FontSize', 7, ...
        'Position', [65, 40, 100, 20], ...
        'Tag', 'ResetKinovaButton', ...
        'Enable', 'off');

    % Store GUI handles in figure's UserData
    guiData.sliderUR3 = sliderUR3;
    guiData.sliderKinova = sliderKinova;
    guiData.statusText = statusText;
    guiData.estopButton = estopButton;
    guiData.startButton = startButton;
    guiData.lightCurtainButton = lightCurtainButton; % Store the new button handle
    set(f, 'UserData', guiData);
end

function launchLightCurtain(~, ~)
    % Callback function to launch lightcurtain.m
    LightCurtain;  % Call the lightcurtain script
end

function startSimulation(src, ~)
    % Get figure and GUI data
    f = ancestor(src, 'figure');
    guiData = get(f, 'UserData');
    
    % Update status
    set(guiData.statusText, 'String', 'Initializing robots...');
    drawnow;
    
    % Create LA2 instance
    robotInstance = LA2();
    
    % Store robot instance in figure's UserData
    guiData.robotInstance = robotInstance;
    set(f, 'UserData', guiData);
    
    % Enable all controls
    enableControls(f, true);
    
    % Set up callbacks now that we have the robot instance
    setupCallbacks(f, robotInstance);
    
    % Change start button to play button
    set(guiData.startButton, 'String', 'Play', ...
        'Callback', @(src,event)playAction(robotInstance));
end

function enableControls(f, enable)
    % Get GUI data
    guiData = get(f, 'UserData');
    
    % Enable/disable all sliders
    for i = 1:6
        set(findobj(f, 'Tag', sprintf('UR3_slider_%d', i)), 'Enable', iif(enable, 'on', 'off'));
        set(findobj(f, 'Tag', sprintf('Kinova_slider_%d', i)), 'Enable', iif(enable, 'on', 'off'));
    end
    
    % Enable/disable buttons
    set(findobj(f, 'Tag', 'EStopButton'), 'Enable', iif(enable, 'on', 'off'));
    set(findobj(f, 'Tag', 'ResetUR3Button'), 'Enable', iif(enable, 'on', 'off'));
    set(findobj(f, 'Tag', 'ResetKinovaButton'), 'Enable', iif(enable, 'on', 'off'));
end

function setupCallbacks(f, robotInstance)
    % Get GUI data
    guiData = get(f, 'UserData');
    
    % Set up UR3 slider callbacks
    for i = 1:6
        slider = findobj(f, 'Tag', sprintf('UR3_slider_%d', i));
        set(slider, 'Callback', @(src,event)updateUR3Joint(src, event, i, robotInstance.robot1));
    end
    
    % Set up Kinova slider callbacks
    for i = 1:6
        slider = findobj(f, 'Tag', sprintf('Kinova_slider_%d', i));
        set(slider, 'Callback', @(src,event)updateKinovaJoint(src, event, i, robotInstance.robot2));
    end
    
    % Set up button callbacks
    set(findobj(f, 'Tag', 'EStopButton'), 'Callback', @(src,event)toggleEStop(robotInstance));
    set(findobj(f, 'Tag', 'ResetUR3Button'), 'Callback', @(src,event)resetUR3(robotInstance.robot1, guiData.sliderUR3));
    set(findobj(f, 'Tag', 'ResetKinovaButton'), 'Callback', @(src,event)resetKinova(robotInstance.robot2, guiData.sliderKinova));
end

function result = iif(condition, trueVal, falseVal)
    if condition
        result = trueVal;
    else
        result = falseVal;
    end
end

% The following functions remain unchanged from your original code
function updateUR3Joint(src, ~, joint, robot)
    valueText = findobj('Tag', sprintf('UR3_value_%d', joint));
    set(valueText, 'String', sprintf('%.1f°', src.Value));
    q = robot.model.getpos();
    q(joint) = deg2rad(src.Value);
    robot.model.animate(q);
end

function updateKinovaJoint(src, ~, joint, robot)
    valueText = findobj('Tag', sprintf('Kinova_value_%d', joint));
    set(valueText, 'String', sprintf('%.1f°', src.Value));
    q = robot.model.getpos();
    q(joint) = deg2rad(src.Value);
    robot.model.animate(q);
end

function resetUR3(robot, sliders)
    q = zeros(1, 6);
    robot.model.animate(q);
    for i = 1:6
        set(sliders(i), 'Value', 0);
        valueText = findobj('Tag', sprintf('UR3_value_%d', i));
        set(valueText, 'String', '0.0°');
    end
end

function resetKinova(robot, sliders)
    q = zeros(1, 6);
    robot.model.animate(q);
    for i = 1:6
        set(sliders(i), 'Value', 0);
        valueText = findobj('Tag', sprintf('Kinova_value_%d', i));
        set(valueText, 'String', '0.0°');
    end
end

function toggleEStop(robotInstance)
    robotInstance.estopFlag = ~robotInstance.estopFlag;
    if robotInstance.estopFlag
        disp('E-Stop Activated: System is stopped.');
    else
        disp('E-Stop Deactivated: System resumed.');
    end
end

function playAction(robotInstance)
    robotInstance.R1();
    pause(1);
    robotInstance.R2();
end