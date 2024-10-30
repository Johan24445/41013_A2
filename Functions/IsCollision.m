%% IsCollision
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
