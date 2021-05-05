load Maps simpleMap
mapMatrix = simpleMap;
bom = binaryOccupancyMap(mapMatrix);
mapScale = 1;
scanAngles = [-3*pi/8 : pi/8 :3*pi/8];
maxRange = 12;

lidarNoiseVariance = 0.1^2;
lidarNoiseSeeds = randi(intmax,size(scanAngles));

posNoiseVariance = 0.1^2;
posNoiseSeeds = randi(intmax,[1, 2]);

angNoiseVariance = 0.01^2;
angNoiseSeeds = randi(intmax);
% Max speed parameters
maxLinSpeed = 0.3;
maxAngSpeed = 0.3;
% Max configuration parameters
Xrange = bom.XWorldLimits;
Yrange = bom.YWorldLimits;
thetaRange = 2*pi;

% Initial pose of the robot
robotX = 23;
robotY = 23;
targetX = 7;
targetY = 4.5;
robotTheta = -pi/2;
targetTheta = -pi/2;

mdl = "MobileRobotObstacleAvoidanceWithTarget";

Tfinal = 100;
sampleTime = 0.1;

agentBlk = mdl + "/Agent";
open_system(mdl)

%%
numStates = numel(scanAngles) + 3;
stateInfo = rlNumericSpec([numStates 1],...
    "LowerLimit",zeros(numStates,1),...
    "UpperLimit",[ones(numel(scanAngles),1)*maxRange; Xrange(2); Yrange(2); thetaRange]);
numObservations = stateInfo.Dimension(1);

numActions = 2;
actInfo = rlNumericSpec([numActions 1],...
    "LowerLimit",-1,...
    "UpperLimit",1);

env = rlSimulinkEnv(mdl,agentBlk,stateInfo,actInfo);
env.ResetFcn = @(in)RLAvoidObstaclesTargetResetFcn(in,scanAngles,maxRange,mapMatrix);

env.UseFastRestart = "Off";

load ('demo/trainedagent_demo.mat');
obstacleAvoidanceAgentTarget = saved_agent;
out=sim('MobileRobotObstacleAvoidanceWithTarget.slx');
showResult;

