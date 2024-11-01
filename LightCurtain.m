%% Simple Light Curtain for UR3 Robot with Moving Hand Detection
% UR3 Robot Setup
axis equal;
hold on;
lc1 = PlaceObject('lightcurtain.ply',[0.77,0.8,-0.46]);
lcRot= pi/2;
scale = 0.65; 
tform = hgtransform;
R_lc = makehgtform('xrotate', lcRot,'scale', scale);
set(tform, 'Matrix', R_lc);
set(lc1, 'Parent', tform);


lc2 = PlaceObject('lightcurtain.ply',[-0.77,0.8,-0.46]);
lcRot= pi/2;
scale = 0.65; 
tform = hgtransform;
R_lc = makehgtform('xrotate', lcRot,'scale', scale);
set(tform, 'Matrix', R_lc);
set(lc2, 'Parent', tform);



r = UR3;  % Initialize UR3 robot
q1 = [0, 0, 0, 0, 0, 0];  % Initial joint configuration
q2 = [pi/2, 0, 0, 0, 0, 0];  % Final joint configuration

r.model.plot(q1);  % Plot UR3 in the initial position
 

%% Light Curtain Setup
% Define a light curtain area using a simple surface plot
[x, z] = meshgrid(-0.5:0.01:0.5, 0:0.01:1);  % X and Z grid for the light curtain
y(1:size(x,1),1:1) = 0.3;  % Fixed Y location for light curtain
lc2 = surf(x, y, z, 'FaceAlpha', 0.1, 'EdgeColor', 'red');  % Create light curtain

%% Load Hand Model
% Load the hand .ply file
[f, v, data] = plyread('hand.ply', 'tri');  % Read hand model
handVertices = v;  % Get vertices of hand

%% Light Curtain Detection Demo
initialHandPosition = transl(0, 0.6, 1);  % Initial hand position (X, Y, Z)
handPlot = trisurf(f, handVertices(:,1), handVertices(:,2), handVertices(:,3), ...
    'FaceColor', 'blue', 'EdgeColor', 'none', 'FaceAlpha', 0.8);  % Visualize hand

% Movement Parameters
steps = 100;  % Number of steps to move the hand
movementDirection = transl(0, -0.005, 0);  % Move in the Y-direction (towards light curtain)

qMatrix = jtraj(q1,q2,steps);
%% Main Loop: Move Hand and Check Light Curtain Detection
for i = 1:steps
    r.model.animate(qMatrix(i,:));
    drawnow();

    % Move the hand slightly along Y-axis in each iteration
    handPosition = initialHandPosition * movementDirection^i;  % Update hand position
    
    % Transform the hand vertices to match the new hand position
    transformedVertices = [handVertices, ones(size(handVertices, 1),1)] * handPosition';
    
    % Update the hand plot with new vertices
    set(handPlot, 'Vertices', transformedVertices(:,1:3));
    handSizeY = max(transformedVertices(:,2)) - min(transformedVertices(:,2));
    curtainOffset = 0.3 + handSizeY;
    % Check if hand enters the light curtain by detecting Y-axis crossing
    if max(transformedVertices(:, 2)) <= curtainOffset  % Check if hand crosses Y=0.2 (light curtain)
        fprintf('EMERGENCY! STOPPING ROBOT.\n');
        set(gcf, 'color', 'r');  % Change figure background to red to indicate detection
        break
    else
        set(gcf, 'color', 'w');  % Reset figure background to white
    end
    
    pause(0.05);  % Pause to simulate real-time movement
    drawnow();  % Update figure
end
