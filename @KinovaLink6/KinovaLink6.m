classdef KinovaLink6 < RobotBaseClass
    properties(Access = public)              
        plyFileNameStem = 'KinovaLink6';
    end
    
    methods
%% Define robot Function 
    function self = KinovaLink6(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end
            self.model.base = self.model.base.T * baseTr;
            
            self.PlotAndColourRobot();         
        end

%% Create the robot model
        function CreateModel(self)   
            % Create the Kinova Link 6 Robot
            link(1) = Link([0,     0.1950,     0.11024,    pi/2]);
            link(2) = Link([0,  0,         -0.485,      pi]);
            link(3) = Link([0,  -0.022447,   0,          pi/2]);
            link(4) = Link([0,     -0.37491,    0,          pi/2]);
            link(5) = Link([0, -0.139879,   0.086,      pi/2]);
            link(6) = Link([0,   0.179028,   0,          0]);
            
            % Incorporate joint limits
            link(1).qlim = [-360 360]*pi/180;
            link(2).qlim = [-90 90]*pi/180;
            link(3).qlim = [-170 170]*pi/180;
            link(4).qlim = [-360 360]*pi/180;
            link(5).qlim = [-360 360]*pi/180;
            link(6).qlim = [-360 360]*pi/180;
        
            link(2).offset = -pi/2;
            link(3).offset = pi/2;
            link(4).offset = -pi/2;
            link(5).offset = pi/2;
            
            self.model = SerialLink(link,'name',self.name);
        end
     
    end
end