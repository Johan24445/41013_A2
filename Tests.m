%% RMRC (Lab 9)   
%% Pick phone and run away
% Load UR3 model
    r = UR3();  

    % Define initial and final joint configurations
    q0 = [0,0,0,deg2rad(-90), deg2rad(-90),0]; % Starting pose\
    q1 = [0,0,deg2rad(10),deg2rad(-90), deg2rad(-90),0]; 
    q2 = [deg2rad(200), 0, 0, deg2rad(-90), deg2rad(-90), 0]; % drops phone 
    q3 = [deg2rad(200), 0,deg2rad(10) , deg2rad(-90), deg2rad(-90), 0];

    stepsMini = 25;

    qMat1 = jtraj(q0,q1,stepsMini);
    qMat2 = jtraj(q2,q3,stepsMini); 
    r.model.animate(qMat1)
    drawnow();


    stepsR1 = 100;  % Number of steps
    deltaT = 0.05;  % Time step
    epsilon = 0.2;  % Threshold for manipulability

% Animation and graphs
    % Precompute the trajectory using a sigmoid time scaling
    sigmoid_time = @(t) 1 ./ (1 + exp(-20*(t - 0.2)));  % Sigmoid scaling
    scaled_t = sigmoid_time(linspace(0, 1, stepsR1));  % Scale the time non-linearly

    steps = 100; 

    % Allocate array data
    m = zeros(steps,1);  
    qMatrix1 = zeros(stepsR1, length(q1));
    % qMatrix1(1,:) = q1; 
    qdot = zeros(stepsR1, 6);  % Array for joint velocities
    x = zeros(3,stepsR1);  % End-effector position
    theta = zeros(3,stepsR1);  % Roll, Pitch, Yaw angles
    positionError = zeros(3,steps); % For plotting trajectory error
    angleError = zeros(3,steps); 

    % Generate trajectory for over-the-head movement
    T1 = r.model.fkine(q1).T;  % Initial end-effector pose
    T2 = r.model.fkine(q2).T;  % Final end-effector pose

    for i = 1:stepsR1
        % Fixed x, y position, movement in z and over-the-head rotation
        x(:,i) = (1 - scaled_t(i)) * T1(1:3, 4) + scaled_t(i) * T2(1:3, 4);
        x(3,i) = x(3,i) + 0.2 * sin(pi * scaled_t(i));  % Overhead movement (z-axis)
        
        theta(1,i) = 0;  % Roll (constant)
        theta(2,i) = pi * scaled_t(i);  % Pitch (over the head)
        theta(3,i) = 0;  % Yaw (constant)
    end

    T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
    q0 = zeros(1,6);                                                            % Initial guess for joint angles
    qMatrix1(1,:) = r.model.ikcon(T,q0);  

    % RMRC Loop
    for i = 1:stepsR1 - 1
        T = r.model.fkine(qMatrix1(i,:)).T;  % Current end-effector pose

        % Calculate position and orientation errors
        deltaX = x(:,i+1) - T(1:3,4);  % Position error
        Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));
        % Rd = rpy2r(theta(:,i+1)');  % Desired orientation as a rotation matrix
        Ra = T(1:3,1:3);  % Current orientation
        Rdot = (1/deltaT) * (Rd - Ra);  % Rotation matrix error
        S = Rdot * Ra'; % Skew-symmetric matrix for angular velocity
        linear_velocity = (1/deltaT) * deltaX;  % Linear velocity
        angular_velocity = [S(3,2); S(1,3); S(2,1)];  % Angular velocity
        deltaTheta = tr2rpy(Rd*Ra'); % Angle errors 

        % Combine linear and angular velocity into a single velocity vector
        xdot = [linear_velocity; angular_velocity];

        % Get the Jacobian at the current joint position
        J = r.model.jacob0(qMatrix1(i,:));
        m(i) = sqrt(det(J*J'));         % Record manipulability

        % Apply Damped Least Squares (DLS) if manipulability is low
       

        if m(i) < epsilon  % If manipulability is less than given threshold
            lambda = (1 - m(i)/epsilon)*5E-2;
        else
            lambda = 0;
        end
            invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse
    

        % Compute joint velocities using the inverse Jacobian
        qdot(i,:) = (invJ * xdot)';  % Joint velocities
        
        for j = 1:6                                                             % Loop through joints 1 to 6
            if qMatrix1(i,j) + deltaT*qdot(i,j) < r.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
                qdot(i,j) = 0; % Stop the motor
            elseif qMatrix1(i,j) + deltaT*qdot(i,j) > r.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                qdot(i,j) = 0; % Stop the motor
            end
        end
        % Update joint positions based on velocities
        qMatrix1(i+1,:) = qMatrix1(i,:) + deltaT * qdot(i,:);  % Next joint configuration
        positionError(:,i) = deltaX;                               
        angleError(:,i) = deltaTheta;  
    end



    % Animate the motion from q1 to q2
    for i = 1:stepsR1
        r.model.animate(qMatrix1(i, :));  % Animate robot
        drawnow();
    end

    pause(0.5);

    % PLOTS FOR MANIPULABILITY AND POSITION/ANGLE ERRORS

for i = 1:6
    figure(2)
    subplot(3,2,i)
    plot(qMatrix(:,i),'k','LineWidth',1)
    title(['Joint ', num2str(i)])
    ylabel('Angle (rad)')
    refline(0,r.model.qlim(i,1));
    refline(0,r.model.qlim(i,2));
    
    figure(3)
    subplot(3,2,i)
    plot(qdot(:,i),'k','LineWidth',1)
    title(['Joint ',num2str(i)]);
    ylabel('Velocity (rad/s)')
    refline(0,0)
end
figure(4)
subplot(2,1,1)
plot(positionError'*1000,'LineWidth',1)
refline(0,0)
xlabel('Step')
ylabel('Position Error (mm)')
legend('X-Axis','Y-Axis','Z-Axis')

subplot(2,1,2)
plot(angleError','LineWidth',1)
refline(0,0)
xlabel('Step')
ylabel('Angle Error (rad)')
legend('Roll','Pitch','Yaw')
figure(5)
plot(m,'k','LineWidth',1)
refline(0,epsilon)
title('Manipulability')

    %% Pick phone and return 
    r.model.animate(qMat2)
    drawnow();
   
% Precompute the trajectory using the scaled time
qMatrix2 =zeros(stepsR1, length(q3));
for i = 1:stepsR1
    % Interpolate joint space positions based on scaled time
    qMatrix2(i, :) = (1 - scaled_t(i)) * q3 + scaled_t(i) * q0;
end

% Animate motion
for i = 1:stepsR1
    r.model.animate(qMatrix2(i, :));
    drawnow();
end
pause(0.5);

%% Collision Detection
r = UR3;
q = zeros(1,6);                                                     % Create a vector of initial joint angles        
scale = 0.5;
workspace = [-2 2 -2 2 -1 2];                                       % Set the size of the workspace when drawing the robot
r.model.plot(q,'workspace',workspace,'scale',scale);                  % Plot the robot
        

centerpnt = [1.2,0,0];
side = 1.5;
plotOptions.plotFaces = true;
[vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);

groundNormal = [0, 0, 1];  % Ground plane normal (z-axis)
groundPoint = [0, 0, -0.001];   % A point on the ground plane (z = 0)
axis equal

% Get the transform of every joint (i.e. start and end of every link)
tr = zeros(4,4,r.model.n+1);
tr(:,:,1) = r.model.base;
L = r.model.links;
for i = 1 : r.model.n
    tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
end

% Go through each link and also each triangle face
for i = 1 : size(tr,3)-1    
    for faceIndex = 1:size(faces,1)
        vertOnPlane = vertex(faces(faceIndex,1)',:);
        [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
        if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
            plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
            disp('Intersection');
        end
    end 
    [groundIntersectP, groundCheck] = LinePlaneIntersection(groundNormal, groundPoint, tr(1:3, 4, i)', tr(1:3, 4, i+1)');
    if (groundCheck == 1) && groundIntersectP(3) == -0.001 % Intersection at z = -0.001
        plot3(groundIntersectP(1), groundIntersectP(2), groundIntersectP(3), 'r*');
        disp(['Intersection with ground at joint ', num2str(i)]);
    end
end



% Go through until there are no step sizes larger than 1 degree
q1 = [deg2rad(90),0,0,0,0,0];
q2 = [deg2rad(270),0,0,0,0,0];
steps = 2;
while ~isempty(find(1 < abs(diff(rad2deg(jtraj(q1,q2,steps)))),1))
    steps = steps + 1;
end
qMatrix = jtraj(q1,q2,steps);

result = true(steps,1);
for i = 1: steps
    result(i) = IsCollision(r.model,qMatrix(i,:),faces,vertex,faceNormals,false);
    r.model.animate(qMatrix(i,:));
end

%% Randomly select waypoints (primative RRT) - Collision Avoidance 
r.model.animate(q1);
qWaypoints = [q1;q2];
isCollision = true;
checkedTillWaypoint = 1;
qMatrix = [];
while (isCollision)
    startWaypoint = checkedTillWaypoint;
    for i = startWaypoint:size(qWaypoints,1)-1
        qMatrixJoin = InterpolateWaypointRadians(qWaypoints(i:i+1,:),deg2rad(10));
        if ~IsCollision(r.model,qMatrixJoin,faces,vertex,faceNormals)
            qMatrix = [qMatrix; qMatrixJoin]; %#ok<AGROW>
            r.model.animate(qMatrixJoin);
            s = size(qMatrix);
            fprintf('Number of steps: %d\n', s);  
            isCollision = false;
            checkedTillWaypoint = i+1;
            % Now try and join to the final goal (q2)
            qMatrixJoin = InterpolateWaypointRadians([qMatrix(end,:); q2],deg2rad(10));
            if ~IsCollision(r.model,qMatrixJoin,faces,vertex,faceNormals)
                qMatrix = [qMatrix;qMatrixJoin];
                % Reached goal without collision, so break out
                break;
            end
        else
            % Randomly pick a pose that is not in collision
            qRand = (2 * rand(1,6) - 1) * pi;
            while IsCollision(r.model,qRand,faces,vertex,faceNormals)
                qRand = (2 * rand(1,6) - 1) * pi;
            end
            qWaypoints =[ qWaypoints(1:i,:); qRand; qWaypoints(i+1:end,:)];
            isCollision = true;
            break;
        end
    end
end
r.model.animate(qMatrix)

%% Laser proximity sensing 
% Initialize UR3 robot model
robot = UR3();
q = [0, pi/2, 0, 0, 0, 0];  % Initial joint configuration
% Create a new figure with specified dimensions
figure('Position', [100, 100, 1200, 800]);  % Adjust [x, y, width, height] for larger size

% Plot the robot again, but now in the larger figure window
robot.model.plot3d(q);

% Adjust the axis limits to zoom out and see more of the scene
xlim([-1, 1.5]);  % Adjust according to your scene
ylim([-1, 1]);    % Adjust according to your scene
zlim([0, 1.5]);   % Adjust according to your scene

hold on;

% Define the cube's initial position and size
cubeCenter = [0.5, 0, 0.5];  % Cube's initial center position (x, y, z)
cubeSize = [0.2, 0.2, 0.2];  % Cube's size (length, width, height)

% Plot the cube
[vertex, faces, faceNormals] = RectangularPrism(cubeCenter - cubeSize / 2, cubeCenter + cubeSize / 2);
patch('Vertices', vertex, 'Faces', faces, 'FaceColor', 'red', 'FaceAlpha', 0.8);
axis equal;

% Desired distance from the cube
desiredDistance = 0.5;  % 0.5 meters
speed = 0.01;           % Speed at which robot adjusts its position

for t = 1:200  % Simulate for 200 iterations (can adjust as necessary)
    % Simulate the cube moving (you can modify this to use your own movement logic)
    cubeCenter = cubeCenter + [0.01 * sin(t * 0.05), 0, 0];  % Move cube in the x-direction
    [vertex, ~, ~] = RectangularPrism(cubeCenter - cubeSize / 2, cubeCenter + cubeSize / 2);
    delete(findobj('FaceColor', 'red'));  % Remove previous cube plot
    patch('Vertices', vertex, 'Faces', faces, 'FaceColor', 'red', 'FaceAlpha', 0.8);  % Re-plot the cube

    % Get the current end-effector position (fkine gives SE3 object, so we access the .T property for matrix)
    tr = robot.model.fkine(q).T;
    endEffectorPos = tr(1:3, 4)';  % Current end-effector position

    % Calculate the vector from the end-effector to the cube's center
    vectorToCube = cubeCenter - endEffectorPos;

    % Calculate the distance between the end-effector and the cube
    distanceToCube = norm(vectorToCube);

    % If the distance is greater than the desired distance, move closer
    if distanceToCube > desiredDistance
        moveDirection = vectorToCube / distanceToCube;  % Normalize the direction vector
        newEndEffectorPos = endEffectorPos + moveDirection * (distanceToCube - desiredDistance) * speed;

        % Calculate the new joint angles using inverse kinematics to reach the new position
        qNew = robot.model.ikcon(transl(newEndEffectorPos), q);  % Use inverse kinematics to find new joint configuration

        % Update robot configuration and animate
        robot.model.animate(qNew);
        q = qNew;  % Update current joint configuration
    end

    % Pause to slow down animation and create real-time effect
    pause(0.05);
end



%% IsIntersectionPointInsideTriangle
% Given a point which is known to be on the same plane as the triangle
% determine if the point is 
% inside (result == 1) or 
% outside a triangle (result ==0 )
function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)

u = triangleVerts(2,:) - triangleVerts(1,:);
v = triangleVerts(3,:) - triangleVerts(1,:);

uu = dot(u,u);
uv = dot(u,v);
vv = dot(v,v);

w = intersectP - triangleVerts(1,:);
wu = dot(w,u);
wv = dot(w,v);

D = uv * uv - uu * vv;

% Get and test parametric coords (s and t)
s = (uv * wv - vv * wu) / D;
if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
    result = 0;
    return;
end

t = (uv * wu - uu * wv) / D;
if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
    result = 0;
    return;
end

result = 1;                      % intersectP is in Triangle
end

%% IsCollision
% This is based upon the output of questions 2.5 and 2.6
% Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
% and triangle obstacles in the environment (faces,vertex,faceNormals)
function result = IsCollision(robot,qMatrix,faces,vertex,faceNormals,returnOnceFound)
if nargin < 6
    returnOnceFound = true;
end
result = false;
groundNormal = [0, 0, 1];  % Ground plane normal (z-axis)
groundPoint = [0, 0, -0.001];   % A point on the ground plane (z = 0)

for qIndex = 1:size(qMatrix,1)
    % Get the transform of every joint (i.e. start and end of every link)
    tr = GetLinkPoses(qMatrix(qIndex,:), robot);

    % Go through each link and also each triangle face
    for i = 1 : size(tr,3)-1    
        for faceIndex = 1:size(faces,1)
            vertOnPlane = vertex(faces(faceIndex,1)',:);
            [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                display('Intersection');
                result = true;
                if returnOnceFound
                    return
                end
            end
        end    
         % Check collision with the ground plane (z = 0)
            [groundIntersectP, groundCheck] = LinePlaneIntersection(groundNormal, groundPoint, tr(1:3, 4, i)', tr(1:3, 4, i+1)');
            if groundCheck == 1 && groundIntersectP(3) == -0.001 % Intersection at z = -0.001
                plot3(groundIntersectP(1), groundIntersectP(2), groundIntersectP(3), 'r*');
                display(['Intersection with ground at joint ', num2str(i)]);  % Display which joint is involved
                result = true;
                if returnOnceFound
                    return
                end
            end


    end
end
end

%% GetLinkPoses
% q - robot joint angles
% robot -  seriallink robot model
% transforms - list of transforms
function [ transforms ] = GetLinkPoses( q, robot)

links = robot.links;
transforms = zeros(4, 4, length(links) + 1);
transforms(:,:,1) = robot.base;

for i = 1:length(links)
    L = links(1,i);
    
    current_transform = transforms(:,:, i);
    
    current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
    transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
    transforms(:,:,i + 1) = current_transform;
end
end

%% FineInterpolation
% Use results from Q2.6 to keep calling jtraj until all step sizes are
% smaller than a given max steps size
function qMatrix = FineInterpolation(q1,q2,maxStepRadians)
if nargin < 3
    maxStepRadians = deg2rad(1);
end
    
steps = 2;
while ~isempty(find(maxStepRadians < abs(diff(jtraj(q1,q2,steps))),1))
    steps = steps + 1;
end
qMatrix = jtraj(q1,q2,steps);
end

%% InterpolateWaypointRadians
% Given a set of waypoints, finely intepolate them
function qMatrix = InterpolateWaypointRadians(waypointRadians,maxStepRadians)
if nargin < 2
    maxStepRadians = deg2rad(1);
end

qMatrix = [];
for i = 1: size(waypointRadians,1)-1
    qMatrix = [qMatrix ; FineInterpolation(waypointRadians(i,:),waypointRadians(i+1,:),maxStepRadians)]; %#ok<AGROW>
end
end