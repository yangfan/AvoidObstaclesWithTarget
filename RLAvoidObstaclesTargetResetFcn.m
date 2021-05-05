function in = RLAvoidObstaclesTargetResetFcn(in,~,maxRange,mapMatrix)
% Reset function for reinforcement learning based obstacle avoidance

    % Load map and lidar sensor (to generate valid pose)
    persistent map lidar
    if isempty(map) && isempty(lidar)
        map = binaryOccupancyMap(mapMatrix);
        lidar = rangeSensor('HorizontalAngle', [-3*pi/8, 3*pi/8], 'HorizontalAngleResolution', pi/8, 'RangeNoise', 0.01, 'Range', [0 maxRange]);
    
    end
    
    %% Fixed configuration of robot and target
    u = [23,23,-pi/2];
    robotX = u(1);
    robotY = u(2);
    robotTheta = u(3);
    targetX = 7;
    targetY = 4.5;
    targetTheta = -pi/2;
    in = setVariable(in,'robotX', robotX);
    in = setVariable(in,'robotY', robotY);
    in = setVariable(in,'robotTheta', robotTheta);
    
    in = setVariable(in,'targetX', targetX);
    in = setVariable(in,'targetY', targetY);
    in = setVariable(in,'targetTheta', targetTheta);
    
    in = setVariable(in,'lidarNoiseSeeds',randi(intmax, lidar.NumReadings, 1));
    in = setVariable(in,'posNoiseSeeds',randi(intmax, 2, 1));
    in = setVariable(in,'angNoiseSeeds',randi(intmax));
   
%     robotRadius = 0.6;
%     mapInflated = copy(binaryOccupancyMap(mapMatrix));
%     inflate(mapInflated,robotRadius);
%     prm = mobileRobotPRM;
%     prm.Map = mapInflated;
%     prm.NumNodes = 500;
%     prm.ConnectionDistance = 2;
%     path = findpath(prm, [robotPose(1), robotPose(2)], [targetPose(1), targetPose(2)]);
%     in = setVariable(in,'path', path);
    
    %% Randomly generate pose inside the map. 
    % If the pose is in an unoccupied space and there are no range readings
    % nearby, assign this pose to the new simulation run
%     posFound = false;   
%     while(~posFound)
%         pos = [diff(map.XWorldLimits)*rand + map.XWorldLimits(1); ...
%                diff(map.YWorldLimits)*rand + map.YWorldLimits(1); ... 
%                2*pi*rand];  
%         ranges = lidar(pos', map);
%         if ~checkOccupancy(map,pos(1:2)') && all(ranges(~isnan(ranges)) >= 0.5)
%             posFound = true;
%             in = setVariable(in,'initX', pos(1));
%             in = setVariable(in,'initY', pos(2));
%             in = setVariable(in,'initTheta', pos(3));
%         end
%     end
%     in = setVariable(in,'lidarNoiseSeeds',randi(intmax, lidar.NumReadings, 1));
%%

end

