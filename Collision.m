function Collision()
    %% Robot and Objects 
    hold on
    axis ([-1.5 1.5 -3 0.5 0 2])
    
    r1= UR3;
    r1.model.plot([0,0,0,0,0,0]);
    % Collision objects 
    centerpnt = [0,-1.5,1];
    side = 2;
    plotOptions.plotFaces = true; % Show/hide cube 
    [vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
    
    groundCenter = [0, 0, -1];  % Center point of the cuboid
    groundSize = [4, 4, 1.999];     % Ground dimensions (wide and thin)
    plotOptions.plotFaces = false; % Show/hide gtound cuboid
    [groundVertex, groundFaces, groundFaceNormals] = RectangularPrism(groundCenter - groundSize / 2, groundCenter + groundSize / 2, plotOptions);
    
    q0 = [0,0,0,deg2rad(-90), deg2rad(-90),0]; % Starting pose
    % q1 = [0,0,deg2rad(5),deg2rad(-90), deg2rad(-90),0]; % Picks up phone
    % q2 = [deg2rad(200), 0, 0, deg2rad(-90), deg2rad(-90), 0]; % Final pose/ drops phone 
    q3 = [deg2rad(200), 0,deg2rad(5) , deg2rad(-90), deg2rad(-90), 0]; % Picks up phone
    
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
    %% 
    
    % Collision avoidance R1
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