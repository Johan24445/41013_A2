hold on
axis equal
view(3);

% Environment & Safety Features 
surf([-4,-4;4,4] ...
,[-4,4;-4,4] ...
,[0.01,0.01;0.01,0.01] ...
,'CData',imread('concrete.jpg') ...
,'FaceColor','texturemap')

scaledFac= 0.01;

phoneInitPos = [-0.46,-0.19,0.5];
phoneFinalPos = [0.46,-0.19,0.5];


phone= PlaceObject('Phone.ply', [phoneInitPos(1)/scaledFac,phoneInitPos(2)/scaledFac,phoneInitPos(3)/scaledFac]);
scaledVerticesPhone = get(phone, 'Vertices') * scaledFac;
set(phone, 'Vertices', scaledVerticesPhone);

barriers =[];
barriers(1) = PlaceObject('barrier1.5x0.2x1m.ply', [0,1,0]);
barriers(2) = PlaceObject('barrier1.5x0.2x1m.ply', [0,1,0]);
barriers(3) = PlaceObject('barrier1.5x0.2x1m.ply', [0,1,0]);
barriers(4) = PlaceObject('barrier1.5x0.2x1m.ply', [0,1.1,0]);

theta = [];
theta(1)= -pi/2;
theta(2)= pi;
theta(3)= 0;
theta(4) = pi/2;
for i =1:length(barriers)
 tform = hgtransform;
 Rz = makehgtform('zrotate', theta(i),'scale', 3);
 set(tform, 'Matrix', Rz);
 set(barriers(i), 'Parent', tform);
end 

% fence = PlaceObject('fenceAssemblyGreenRectangle4x8x2.5m.ply');

fireEx = PlaceObject('fireExtinguisherElevated.ply', [-0.5,2.5,1]);
scaleFactor = 2;  
scaledVertices = get(fireEx, 'Vertices') * scaleFactor;
set(fireEx, 'Vertices', scaledVertices);

emergButton = PlaceObject('emergencyStopWallMounted.ply', [-0.25,-1.25,0.5]);
emerRot= pi;
tform = hgtransform;
R_emer = makehgtform('zrotate', emerRot,'scale', 4);
set(tform, 'Matrix', R_emer);
set(emergButton, 'Parent', tform);


person = PlaceObject('personMaleConstruction.ply', [-4.8,0,0]);
personRot = -pi/2;
tform = hgtransform;
R_person = makehgtform('zrotate', personRot,'scale', 1);
set(tform, 'Matrix', R_person);
set(person, 'Parent', tform);



% Tables 
tableR1  = PlaceObject('tableBrown2.1x1.4x0.5m.ply',[0,0.3,0]);
tableBed  = PlaceObject('tableBrown2.1x1.4x0.5m.ply', [0,2,0]);
tableR2 = PlaceObject('tableRound0.3x0.3x0.3m.ply', [-0.6,-0.55,-0.1]);

tableMainRot = pi/2;
tform = hgtransform;
R_person = makehgtform('zrotate', tableMainRot,'scale', 1);
set(tform, 'Matrix', R_person);
set(tableBed, 'Parent', tform);
scaleFactor = 3;  
scaledVertices = get(tableR2, 'Vertices') * scaleFactor;
set(tableR2, 'Vertices', scaledVertices);

% People 
me = PlaceObject('personMaleOld.ply',[-2,-0.35,-1.2]);
meRot = deg2rad(-100);
tform = hgtransform;
R_me = makehgtform('xrotate', meRot,'scale', 1);
set(tform, 'Matrix', R_me);
set(me, 'Parent', tform);

% Collision objects 
centerpnt = [0,-1.5,0];
side = 2;
plotOptions.plotFaces = true; % Set this to false to hide cube from plot

[vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);

groundCenter = [0, 0, 0.25];  % Center point of the cuboid
groundSize = [4, 4, 0.49];     % Ground dimensions (wide and thin)
plotOptions.plotFaces = false;
[groundVertex, groundFaces, groundFaceNormals] = RectangularPrism(groundCenter - groundSize / 2, groundCenter + groundSize / 2, plotOptions);

% groundNormal = [0, 0, 1];  % Ground plane normal (z-axis)
% groundPoint = [0, 0, 0.49];   % A point on the ground plane (z = 0)

% Create R1 and R2
r1 = UR3;
r1BaseTransform = transl(0,0, 0.5);
r1.model.base = r1BaseTransform;
r1.model.plot(zeros(1, r1.model.n))

r2 = KinovaLink6;    
r2BaseTransform = transl(-2.25, -1.5, 0.6)* trotz(pi/2  );
r2.model.base = r2BaseTransform;    
r2.model.plot(zeros(1, r2.model.n))

%% Activate & Animate R1
q0 = [0,0,0,deg2rad(-90), deg2rad(-90),0]; % Starting pose
q1 = [0,0,deg2rad(5),deg2rad(-90), deg2rad(-90),0]; 
q2 = [deg2rad(200), 0, 0, deg2rad(-90), deg2rad(-90), 0]; % drops phone 
q3 = [deg2rad(200), 0,deg2rad(5) , deg2rad(-90), deg2rad(-90), 0];

stepsMini = 25;
stepsR1 = 100;  % Number of steps
deltaT = 0.05;  % Time step
epsilon = 0.2;  % Threshold for manipulability

qMat1 = jtraj(q0,q1,stepsMini);
qMat2 = jtraj(q2,q3,stepsMini); 
r1.model.animate(qMat1)
drawnow();

% Phone handles 
robotPhoneOffset = 0.07;
tPhoneStart = transl(phoneInitPos(1),phoneInitPos(2),phoneInitPos(3)+robotPhoneOffset);
tPhoneEnd = transl(phoneFinalPos(1),phoneFinalPos(2),phoneFinalPos(3)+robotPhoneOffset);

q1 = r1.model.ikine(tPhoneStart,'mask', [1 1 1 0 0 0]);
q2 = r1.model.ikine(tPhoneEnd,'mask', [1 1 1 0 0 0]);

% Time scaling to speed up near bed 

sigmoid_time = @(t) 1 ./ (1 + exp(-20*(t - 0.2)));  % Sigmoid scaling
scaled_t = sigmoid_time(linspace(0, 1, stepsR1));     % Scale the time non-linearly

% Allocate array data
    m = zeros(stepsR1,1);  
    qMatrix1 = zeros(stepsR1, length(q1));
    % qMatrix1(1,:) = q1; 
    qdot = zeros(stepsR1, 6);  % Array for joint velocities
    x = zeros(3,stepsR1);  % End-effector position
    theta = zeros(3,stepsR1);  % Roll, Pitch, Yaw angles
    positionError = zeros(3,stepsR1); % For plotting trajectory error
    angleError = zeros(3,stepsR1); 

    % Generate trajectory for over-the-head movement
    T1 = r1.model.fkine(q1).T;  % Initial end-effector pose
    T2 = r1.model.fkine(q2).T;  % Final end-effector pose

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
    qMatrix1(1,:) = r1.model.ikcon(T,q0);  

    % RMRC Loop
    for i = 1:stepsR1 - 1
        T = r1.model.fkine(qMatrix1(i,:)).T;  % Current end-effector pose

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
        J = r1.model.jacob0(qMatrix1(i,:));
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
            if qMatrix1(i,j) + deltaT*qdot(i,j) < r1.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
                qdot(i,j) = 0; % Stop the motor
            elseif qMatrix1(i,j) + deltaT*qdot(i,j) > r1.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
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
        r1.model.animate(qMatrix1(i, :));  % Animate robot
        drawnow();
        currentQ_R1 = qMatrix1(i, :);
        currentPos_R1 = r1.model.fkine(currentQ_R1).T; % End effector pose of robot during motion
        currentPos_R1 = currentPos_R1(1:3, 4);
        currentPos_R1(3) = currentPos_R1(3) - robotPhoneOffset;
        phonePosUpdated = currentPos_R1';
        delete(phone); 
        phone = PlaceObject('phone.ply', [phonePosUpdated(1)/scaledFac,phonePosUpdated(2)/scaledFac,phonePosUpdated(3)/scaledFac]);
        scaledVerticesPhone = get(phone, 'Vertices') * scaledFac;
        set(phone, 'Vertices', scaledVerticesPhone);
    end

    pause(0.5);

     % Pick phone and return (q3->q0)
r1.model.animate(qMat2)
drawnow();

% Precompute the trajectory using the scaled time
qMatrix2 =zeros(stepsR1, length(q3));
for i = 1:stepsR1
    % Interpolate joint space positions based on scaled time
    qMatrix2(i, :) = (1 - scaled_t(i)) * q3 + scaled_t(i) * q0;
end

% Animate motion
for i = 1:stepsR1
    r1.model.animate(qMatrix2(i, :));
    drawnow();
    currentQ_R1 = qMatrix2(i, :);
    currentPos_R1 = r1.model.fkine(currentQ_R1).T; % End effector pose of robot during motion
    currentPos_R1 = currentPos_R1(1:3, 4);
    currentPos_R1(3) = currentPos_R1(3) - robotPhoneOffset;
    phonePosUpdated = currentPos_R1';
    delete(phone); 
    phone = PlaceObject('phone.ply', [phonePosUpdated(1)/scaledFac,phonePosUpdated(2)/scaledFac,phonePosUpdated(3)/scaledFac]);
    scaledVerticesPhone = get(phone, 'Vertices') * scaledFac;
    set(phone, 'Vertices', scaledVerticesPhone);
end
pause(0.5);

%% Collision detection R1
q0 = [0,0,0,deg2rad(-90), deg2rad(-90),0]; % Starting pose
q1 = [0,0,deg2rad(5),deg2rad(-90), deg2rad(-90),0]; 
q2 = [deg2rad(200), 0, 0, deg2rad(-90), deg2rad(-90), 0]; % drops phone 
q3 = [deg2rad(200), 0,deg2rad(5) , deg2rad(-90), deg2rad(-90), 0];

% q0 = [0,deg2rad(45),0,deg2rad(-90), deg2rad(-90),0]; % Ground CD
q = zeros(1,6);  

tr1 = zeros(4,4,r1.model.n+1);
tr1(:,:,1) = r1.model.base;
L = r1.model.links;
for i = 1 : r1.model.n
    tr1(:,:,i+1) = tr1(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
end

steps = 2;
while ~isempty(find(1 < abs(diff(rad2deg(jtraj(q3,q0,steps)))),1))
    steps = steps + 1;
end
qMatrixCD = jtraj(q3,q0,steps);

result = true(steps,1);
for i = 1: steps
    result(i) = IsCollision(r1.model,true, qMatrixCD(i,:),faces,vertex,faceNormals,false, groundVertex, groundFaces, groundFaceNormals);
    r1.model.animate(qMatrixCD(i,:));
end
figure;
plot(1:steps, result, 'r-o', 'LineWidth', 2, 'MarkerSize', 5);
xlabel('Step Number');
ylabel('Collision Detected (1 = Yes, 0 = No)');
title('Collision Detection Over Robot Path');
grid on;

%% Collision avoidance R1
qWaypoints = [q3;q0];
isCollision = true;
checkedTillWaypoint = 1;
qMatrixCA = [];
while (isCollision)
    startWaypoint = checkedTillWaypoint;
    for i = startWaypoint:size(qWaypoints,1)-1
        qMatrixJoin = InterpolateWaypointRadians(qWaypoints(i:i+1,:),deg2rad(10));
        if ~IsCollision(r1.model,false, qMatrixJoin,faces,vertex,faceNormals, false, groundVertex, groundFaces, groundFaceNormals)
            qMatrixCA = [qMatrixCA; qMatrixJoin]; %#ok<AGROW> 
            isCollision = false;
            checkedTillWaypoint = i+1;
            % Try and join to the final goal (q0)
            qMatrixJoin = InterpolateWaypointRadians([qMatrixCA(end,:); q0],deg2rad(10));
            if ~IsCollision(r1.model,false, qMatrixJoin,faces,vertex,faceNormals,false, groundVertex, groundFaces, groundFaceNormals)
                qMatrixCA = [qMatrixCA;qMatrixJoin];
                % Reached goal without collision, so break out
                break;
            end
        else
            % Randomly pick a pose that is not in collision
            qRand = (2 * rand(1,6) - 1) * pi;
            while IsCollision(r1.model,false,qRand,faces,vertex,faceNormals, false, groundVertex, groundFaces, groundFaceNormals)
                qRand = (2 * rand(1,6) - 1) * pi;
            end
            qWaypoints =[ qWaypoints(1:i,:); qRand; qWaypoints(i+1:end,:)];
            isCollision = true;
            break;
        end
    end
end
r1.model.animate(qMatrixCA)
s = size(qMatrixCA,1);
fprintf('Number of steps: %d\n', s); 

%% Activate R2
% Define start and target poses (both position and orientation)
targetPose = transl(-2.25, -1, 0.8) * trotx(pi); % Target pose with orientation

% Get the current joint configuration using getpos()
qStart = r2.model.getpos(); % Get current joint angles

% Use inverse kinematics to find the target joint angles
qTarget = r2.model.ikcon(targetPose, qStart); % Target joint config

% Set the number of steps for the trajectory
numSteps = 25;

% Generate a joint trajectory between current and target configurations
qTrajectory = jtraj(qStart, qTarget, numSteps);


for repeat = 1:3    
    % Animate the robot following the joint trajectory
    for i = 1:numSteps
        r2.model.animate(qTrajectory(i, :));
    end
    
    % Return to the start position for the next repetition
    for i = numSteps:-1:1
        r2.model.animate(qTrajectory(i, :));
    end
end
%%  IsIntersectionPointInsideTriangle
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
function result = IsCollision(robot,detecting,qMatrix,faces,vertex,faceNormals,returnOnceFound, groundVertex, groundFaces, groundFaceNormals)
if nargin < 7
    returnOnceFound = true;
end
result = false;

for qIndex = 1:size(qMatrix,1)
    % Get the transform of every joint (i.e. start and end of every link)
    tr = GetLinkPoses(qMatrix(qIndex,:), robot);

    % Go through each link and also each triangle face of cube 
    for i = 1 : size(tr,3)-1    
        for faceIndex = 1:size(faces,1)
            vertOnPlane = vertex(faces(faceIndex,1)',:);
            [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                if detecting
                 plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                 display('Collision with cube');
                end
                result = true;
                if returnOnceFound
                    return
                end
            end
        end  
    end 
         % Check for collision with the ground cuboid
     for i = 1 : size(tr, 3) - 1    
            for faceIndex = 1:size(groundFaces, 1)
                vertOnGroundPlane = groundVertex(groundFaces(faceIndex, 1)', :);
                [groundIntersectP, groundCheck] = LinePlaneIntersection(groundFaceNormals(faceIndex, :), vertOnGroundPlane, tr(1:3, 4, i)', tr(1:3, 4, i+1)');
                if groundCheck == 1 && IsIntersectionPointInsideTriangle(groundIntersectP, groundVertex(groundFaces(faceIndex, :)', :))
                    if detecting
                     plot3(groundIntersectP(1), groundIntersectP(2), groundIntersectP(3), 'r*');
                     display('Collision with ground');
                    end 
                    result = true;
                    if returnOnceFound
                        return;
                    end
                end
            end
     end

     % Check for self-collision between non-adjacent links
        for link1 = 1:size(tr, 3) - 2
            for link2 = link1 + 2:size(tr, 3) - 1  % Avoid adjacent links
                % Get the start and end points of both links
                link1Start = tr(1:3, 4, link1)';
                link1End = tr(1:3, 4, link1+1)';
                link2Start = tr(1:3, 4, link2)';
                link2End = tr(1:3, 4, link2+1)';
                
                % Check for intersection between the two links
                [intersectP, check] = LineSegmentIntersection(link1Start, link1End, link2Start, link2End);
                
                if check == 1
                    % A self-collision is detected
                    if detecting
                     plot3(intersectP(1), intersectP(2), intersectP(3), 'b*');
                     display(['Self-collision detected between links ', num2str(link1), ' and ', num2str(link2)]);
                    end
                    result = true;
                    if returnOnceFound
                        return;
                    end
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
