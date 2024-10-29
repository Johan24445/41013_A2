classdef LA2 <handle 
   properties (Constant)
        robot1= UR3;
        robot2= KinovaLink6;
        phoneInitPos = [-0.46,-0.19,0.5];
        phoneFinalPos = [0.46,-0.19,0.5];
        q0 = [0,0,0,deg2rad(-90), deg2rad(-90),0]; % Starting pose
        q1 = [0,0,deg2rad(5),deg2rad(-90), deg2rad(-90),0]; % Picks up phone
        q2 = [deg2rad(200), 0, 0, deg2rad(-90), deg2rad(-90), 0]; % Final pose/ drops phone 
        q3 = [deg2rad(200), 0,deg2rad(5) , deg2rad(-90), deg2rad(-90), 0]; % Picks up phone
   end

  properties
       estopFlag = false;          % E-Stop flag
       qCurrentR1 = [];            % Store current joint config of R1 during E-Stop
       qCurrentR2 = [];            % Store current joint config of R2 during E-Stop
       joystick;                   % Game controller handle
       buttonIndex = 1;            % Button index for E-Stop (adjust based on controller)
       prevButtonState = false;    % Track previous button state for toggle functionality
   end

   methods
       function self = LA2()
            % cla;
            % clc;
			% input('Press enter to begin')
            self.Setup();
            % self.joystick = vrjoystick(1); % Initialize joystick (controller #1)
			self.R1();
            self.R2();
 
       end
       
       %% E-Stop
       function checkEStop(self)
            % Toggle estopFlag only when button is pressed and released
            buttonPressed = button(self.joystick, self.buttonIndex);
            if buttonPressed && ~self.prevButtonState
                self.estopFlag = ~self.estopFlag; % Toggle E-Stop state
                disp(['E-Stop is now ', num2str(self.estopFlag)]);
            end
            self.prevButtonState = buttonPressed; % Update previous button state
       end

%% Activate and Animate R1
        function R1(self)
        % Create Phone 
        phoneInitPos = LA2.phoneInitPos;
        phoneFinalPos = LA2.phoneFinalPos;
        scaledFacPhone= 0.01;
        phone = PlaceObject('Phone.ply', [phoneInitPos(1)/scaledFacPhone,phoneInitPos(2)/scaledFacPhone,phoneInitPos(3)/scaledFacPhone]);
        scaledVerticesPhone = get(phone, 'Vertices') * scaledFacPhone;
        set(phone, 'Vertices', scaledVerticesPhone);

        r1 = LA2.robot1;
        r1BaseTransform = transl(0,0, 0.5);
        r1.model.base = r1BaseTransform;
        r1.model.plot(zeros(1, r1.model.n))
        q0 = LA2.q0;
        q1 = LA2.q1;
        q2 = LA2.q2;
        q3 = LA2.q3;

        centerpnt = [0,-1.5,0];
        side = 2;
        plotOptions.plotFaces = false; % Set this to false to hide cube from plot
        [vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
        groundCenter = [0, 0, 0.25];  % Center point of the cuboid
        groundSize = [4, 4, 0.49];     % Ground dimensions (wide and thin)
        plotOptions.plotFaces = false;
        [groundVertex, groundFaces, groundFaceNormals] = RectangularPrism(groundCenter - groundSize / 2, groundCenter + groundSize / 2, plotOptions);
        
        stepsMini = 25;
        stepsR1 = 100;  % Number of steps
        deltaT = 0.05;  % Time step
        epsilon = 0.2;  % Threshold for manipulability
        
        qMat1 = jtraj(q0,q1,stepsMini);
        qMat2 = jtraj(q2,q3,stepsMini); 
        r1.model.animate(qMat1)
        drawnow();
        
        % Phone handles 
        robotPhoneOffset = 0.00;
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
                x(:,i) = (1 - scaled_t(i)) * T1(1:3, 4) + scaled_t(i) * T2(1:3, 4);
                x(3,i) = x(3,i) + 0.2 * sin(pi * scaled_t(i));  
        
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

             % Precompute the trajectory using the scaled time
            qMatrix2 =zeros(stepsR1, length(q3));
            for i = 1:stepsR1
                % Interpolate joint space positions based on scaled time
                qMatrix2(i, :) = (1 - scaled_t(i)) * q3 + scaled_t(i) * q0;
            end
        
              % Animate the motion from q1 to q2
                isCollision = true;
                qMatrix1CA = [];
                checkedTillWaypoint = 1;
                while (isCollision)
                    startWaypoint = checkedTillWaypoint;
                    for i = startWaypoint:size(qMatrix1,1)-1
                        qMatrixJoin = InterpolateWaypointRadians(qMatrix1(i:i+1,:),deg2rad(2));
                        if ~IsCollision(r1.model,false, qMatrixJoin,faces,vertex,faceNormals, false, groundVertex, groundFaces, groundFaceNormals)
                            qMatrix1CA = [qMatrix1CA; qMatrixJoin]; %#ok<AGROW> 
                            isCollision = false;
                            checkedTillWaypoint = i+1;
                            % Try and join to the final goal (q0)
                            qMatrixJoin = InterpolateWaypointRadians([qMatrix1CA(end,:); qMatrix1(end,:)],deg2rad(2));
                            if ~IsCollision(r1.model,false, qMatrixJoin,faces,vertex,faceNormals,false, groundVertex, groundFaces, groundFaceNormals)
                                qMatrix1CA = [qMatrix1CA;qMatrixJoin];
                                % Reached goal without collision, so break out
                                break;
                            end
                        else
                            % Randomly pick a pose that is not in collision
                            qRand = (2 * rand(1,6) - 1) * pi;
                            while IsCollision(r1.model,false,qRand,faces,vertex,faceNormals, false, groundVertex, groundFaces, groundFaceNormals)
                                qRand = (2 * rand(1,6) - 1) * pi;
                            end
                            qMatrix1 =[ qMatrix1(1:i,:); qRand; qMatrix1(i+1:end,:)];
                            isCollision = true;
                            break;
                        end
                    end
                end
                        
            for i = 1:size(qMatrix1,1)
                r1.model.animate(qMatrix1CA(i, :));  % Animate robot
                drawnow();
                currentQ_R1 = qMatrix1CA(i, :);
                currentPos_R1 = r1.model.fkine(currentQ_R1).T; % End effector pose of robot during motion
                currentPos_R1 = currentPos_R1(1:3, 4);
                currentPos_R1(3) = currentPos_R1(3) - robotPhoneOffset;
                phonePosUpdated = currentPos_R1';
                delete(phone); 
                phone = PlaceObject('phone.ply', [phonePosUpdated(1)/0.01,phonePosUpdated(2)/0.01,phonePosUpdated(3)/0.01]);
                scaledVerticesPhone = get(phone, 'Vertices') * 0.01;
                set(phone, 'Vertices', scaledVerticesPhone);
            end
            delete(phone);
            phone = PlaceObject('phone.ply', [phonePosUpdated(1)/0.01,phonePosUpdated(2)/0.01,(phonePosUpdated(3)-0.05)/0.01]);
            scaledVerticesPhone = get(phone, 'Vertices') * 0.01;
            set(phone, 'Vertices', scaledVerticesPhone);
                pause(0.5);
            
            % Picks phone (q2 to q3)
            r1.model.animate(qMat2)
            drawnow();
            
            
            % Animate motion from q3 to q0
            for i = 1:stepsR1
                r1.model.animate(qMatrix2(i, :));
                drawnow();
                currentQ_R1 = qMatrix2(i, :);
                currentPos_R1 = r1.model.fkine(currentQ_R1).T; % End effector pose of robot during motion
                currentPos_R1 = currentPos_R1(1:3, 4);
                currentPos_R1(3) = currentPos_R1(3) - robotPhoneOffset;
                phonePosUpdated = currentPos_R1';
                delete(phone); 
                phone = PlaceObject('phone.ply', [phonePosUpdated(1)/0.01,phonePosUpdated(2)/0.01,phonePosUpdated(3)/0.01]);
                scaledVerticesPhone = get(phone, 'Vertices') * 0.01;
                set(phone, 'Vertices', scaledVerticesPhone);
            end
            % Phone drops
            pause(0.5);
            delete(phone);
            phone = PlaceObject('phone.ply', [phonePosUpdated(1)/0.01,phonePosUpdated(2)/0.01,(phonePosUpdated(3)-0.05)/0.01]);
            scaledVerticesPhone = get(phone, 'Vertices') * 0.01;
            set(phone, 'Vertices', scaledVerticesPhone);
          end 
%% Activate & Animate R2
          function R2(self)
           r2 = LA2.robot2;
           r2BaseTransform = transl(-2.25, -1.5, 0.6) * trotz(pi/2);
           r2.model.base = r2BaseTransform;
           r2.model.plot(zeros(1, r2.model.n));
           
           % Define start and target poses
           targetPose = transl(-2.25, -1, 0.8) * trotx(pi);
           qStart = r2.model.getpos();
           qTarget = r2.model.ikcon(targetPose,         qStart);
           numSteps = 25;
           qTrajectory = jtraj(qStart, qTarget, numSteps);
           
           for repeat = 1:3
               for i = 1:numSteps
                   self.checkEStop();
                   if self.estopFlag
                       disp('E-Stop activated for R2. Pausing...');
                       self.qCurrentR2 = qTrajectory(i, :);
                       while self.estopFlag
                           self.checkEStop();
                           pause(0.1);
                       end
                       disp('Resuming R2 from E-Stop position.');
                   end
                   r2.model.animate(qTrajectory(i, :));
               end
               % Return to start
               for i = numSteps:-1:1
                   self.checkEStop();
                   if self.estopFlag
                       disp('E-Stop activated for R2. Pausing...');
                       self.qCurrentR2 = qTrajectory(i, :);
                       while self.estopFlag
                           self.checkEStop();
                           pause(0.1);
                       end
                       disp('Resuming R2 from E-Stop position.');
                   end
                   r2.model.animate(qTrajectory(i, :));
               end
           end
          end       
   end

   methods(Static)
%% Environmental & Robots setup
     function Setup()      
        hold on
        axis ([-4 4 -4 6 0 4]);
        
       surf([-4,-4;4,4] ...
        ,[-4,6;-4,6] ...
        ,[0.01,0.01;0.01,0.01] ...
        ,'CData',imread('concrete.jpg') ...
        ,'FaceColor','texturemap')

    
        % Safety Features 
        barriers =[];
        barriers(1) = PlaceObject('barrier1.5x0.2x1m.ply', [0,1,0]);
        barriers(2) = PlaceObject('barrier1.5x0.2x1m.ply', [0,1,0]);
        barriers(3) = PlaceObject('barrier1.5x0.2x1m.ply', [0,1,0]);
        barriers(4) = PlaceObject('barrier1.5x0.2x1m.ply', [0,1.1,0]);
        
        thetaRotBarrier = [];
        thetaRotBarrier(1)= -pi/2;
        thetaRotBarrier(2)= pi;
        thetaRotBarrier(3)= 0;
        thetaRotBarrier(4) = pi/2;
        for i =1:length(barriers)
         tform = hgtransform;
         Rz = makehgtform('zrotate', thetaRotBarrier(i),'scale', 3);
         set(tform, 'Matrix', Rz);
         set(barriers(i), 'Parent', tform);
        end 
        
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
    
        personWatch = PlaceObject('personMaleConstruction.ply', [-4.8,0,0]);
        personWatchRot = -pi/2;
        tform = hgtransform;
        R_person = makehgtform('zrotate', personWatchRot,'scale', 1);
        set(tform, 'Matrix', R_person);
        set(personWatch, 'Parent', tform);
        
        % Collision objects 
        centerpnt = [0,-1.5,0];
        side = 2;
        plotOptions.plotFaces = false; % Set this to false to hide cube from plot
        
        [vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
        
        groundCenter = [0, 0, 0.25];  % Center point of the cuboid
        groundSize = [4, 4, 0.49];     % Ground dimensions (wide and thin)
        plotOptions.plotFaces = false;
        [groundVertex, groundFaces, groundFaceNormals] = RectangularPrism(groundCenter - groundSize / 2, groundCenter + groundSize / 2, plotOptions);
        
        % Create R1 and R2
        r1 = LA2.robot1;
        r1BaseTransform = transl(0,0, 0.5);
        r1.model.base = r1BaseTransform;
        r1.model.plot(zeros(1, r1.model.n))
        plotOptions.plotFaces = true;
        
        r2 = LA2.robot2;  
        r2BaseTransform = transl(-2.25, -1.5, 0.6)* trotz(pi/2  );
        r2.model.base = r2BaseTransform;    
        r2.model.plot(zeros(1, r2.model.n))
        plotOptions.plotFaces = true;
     end
%% Collision Detection
             function CollisionDetect()
                r1 = LA2.robot1; 
                r1BaseTransform = transl(0,0, 0.5);
                r1.model.base = r1BaseTransform;
                r1.model.plot(zeros(1, r1.model.n))
                q0 = LA2.q0;
                % q1 = LA2.q1;
                % q2 = LA2.q2;
                q3 = LA2.q3;
                % q0 = [0,deg2rad(45),0,deg2rad(-90), deg2rad(-90),0]; % Ground CD

                centerpnt = [0,-1.5,0];
                side = 2;
                plotOptions.plotFaces = true; % Set this to false to hide cube from plot
                [vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
                groundCenter = [0, 0, 0.25];  % Center point of the cuboid
                groundSize = [4, 4, 0.49];     % Ground dimensions (wide and thin)
                plotOptions.plotFaces = false;
                [groundVertex, groundFaces, groundFaceNormals] = RectangularPrism(groundCenter - groundSize / 2, groundCenter + groundSize / 2, plotOptions);

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
             end

%% Collision Avoidance 
             function CollisionAvoid()
                r1 = LA2.robot1;
                r1BaseTransform = transl(0,0, 0.5);
                r1.model.base = r1BaseTransform;
                r1.model.plot(zeros(1, r1.model.n))
                q0 = LA2.q0;
                % q1 = LA2.q1;
                % q2 = LA2.q2;
                q3 = LA2.q3;

                centerpnt = [0,-1.5,0];
                side = 2;
                plotOptions.plotFaces = true; % Set this to false to hide cube from plot
                [vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
                groundCenter = [0, 0, 0.25];  % Center point of the cuboid
                groundSize = [4, 4, 0.49];     % Ground dimensions (wide and thin)
                plotOptions.plotFaces = false;
                [groundVertex, groundFaces, groundFaceNormals] = RectangularPrism(groundCenter - groundSize / 2, groundCenter + groundSize / 2, plotOptions);

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
             end

   end
end
