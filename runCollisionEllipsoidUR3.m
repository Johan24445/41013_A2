%% Test collision
function runCollisionEllipsoidUR3()
    axis ([-1.5 0.8 -1 1 0 0.8])
    hold on
    r = UR3; 
    steps = 25;
    q1 = [0,0,0,0,0,0];
    q2 = [pi/2,0,0,0,0,0];
    qMatrix = jtraj(q1,q2,steps);
    plotOptions = {'noarrow', 'workspace', [-1.5 0.8 -1 1 0 0.8], ...
               'view', [30, 30], 'floorlevel', 0.0};

    % Plot with transparency
    r.model.plot(q1, plotOptions{:}, 'linkcolor', 'none', 'facealpha', 0.1);
    
    
    centerPoints = [0.0, 0.0, 0.05; % Base 
                    0.0, -0.01, 0.01; % Link 1 
                    0.125, 0.0, 0.125; % Link 2 
                    0.105, 0.0, 0.05; % Link 3 
                    0.0, 0.0, 0.01; % Link 4 
                    0.0, 0.0, 0.06; % Link 5 
                    0.0, 0.0, 0.0;]; % end-effector
                
    radii = [0.2, 0.1, 0.15;  
             0.2, 0.1, 0.15;
             0.2, 0.1, 0.15; 
             0.2, 0.1, 0.15; 
             0.2, 0.1, 0.15;
             0.2, 0.1, 0.15; 
             0.0, 0.0, 0.0;]; 
         
    
     
     collision = CollisionEllipsoid(r, centerPoints, radii);
     collision.plotEllipsoids();
     
    
     initMeshPosition = [-1.3,0,0.1];
     mesh = Mesh(0.2,20,initMeshPosition);
     % mesh.updatePlot();
     % Movement parameters for the mesh
     meshMovement = transl(0.08, 0, 0); % Movement vector (e.g., moving in -X direction)
     
     
    
    for i = 1:steps
        % Moves robot
        r.model.animate(qMatrix(i,:))
        drawnow();

        % Move the mesh incrementally
        newMeshPosition = transl(initMeshPosition)  * meshMovement^i;
        newMeshPosition= newMeshPosition(1:3,4)';
        mesh.move(newMeshPosition);  % Update the mesh position in 3D space
        mesh.updateParameters(0.2,10,newMeshPosition)
    
        % Get the updated mesh points and check for collisions with the robot
        meshPoints = mesh.getPoints();
        collisionDetected = collision.checkCollision(meshPoints);
        
        % If a collision is detected, display a warning and break the loop
        if collisionDetected
            fprintf('Collision detected at step: %d \n', i);
            break;
        end
        
        pause(0.1); 
        drawnow;   
       
    end
    
end 

