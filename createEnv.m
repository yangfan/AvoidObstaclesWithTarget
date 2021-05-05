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

fig = figure("Name","simpleMap");
set(fig, "Visible", "on");
ax = axes(fig);

show(binaryOccupancyMap(mapMatrix),"Parent",ax);
hold on
plotTransforms([robotX, robotY, 0], eul2quat([robotTheta, 0, 0]), "MeshFilePath","groundvehicle.stl", "View", "2D");
plotTransforms([targetX, targetY, 0], eul2quat([targetTheta, 0, 0]), "MeshFilePath","multirotor.stl", "View", "2D");
light;
hold off

mdl = "MobileRobotObstacleAvoidanceWithTarget";
% mdl = "MobileRobotObstacleAvoidanceWithTargetSimplified";


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
% reset function for MobileRobotObstacleAvoidanceWithTargetSimplified model
% env.ResetFcn = @(in)RLAvoidObstaclesResetFcn(in,scanAngles,maxRange,mapMatrix);
env.ResetFcn = @(in)RLAvoidObstaclesTargetResetFcn(in,scanAngles,maxRange,mapMatrix);

env.UseFastRestart = "Off";

%%

statePath = [
    featureInputLayer(numObservations, "Normalization", "none", "Name", "State")
    fullyConnectedLayer(60, "Name", "CriticStateFC1")
    reluLayer("Name", "CriticRelu1")
    fullyConnectedLayer(30, "Name", "CriticStateFC2")
    reluLayer("Name", "CriticRelu2")
    fullyConnectedLayer(30, "Name", "CriticStateFC3")];
actionPath = [
    featureInputLayer(numActions, "Normalization", "none", "Name", "Action")
    fullyConnectedLayer(30, "Name", "CriticActionFC1")];
commonPath = [
    additionLayer(2,"Name", "add")
    reluLayer("Name","CriticCommonRelu")
    fullyConnectedLayer(1, "Name", "CriticOutput")];

criticNetwork = layerGraph();
criticNetwork = addLayers(criticNetwork,statePath);
criticNetwork = addLayers(criticNetwork,actionPath);
criticNetwork = addLayers(criticNetwork,commonPath);
criticNetwork = connectLayers(criticNetwork,"CriticStateFC3","add/in1");
criticNetwork = connectLayers(criticNetwork,"CriticActionFC1","add/in2");

actorNetwork = [
    featureInputLayer(numObservations, "Normalization", "none", "Name", "State")
    fullyConnectedLayer(60, "Name", "actorFC1")
    reluLayer("Name","actorReLU1")
    fullyConnectedLayer(60, "Name", "actorFC2")
    reluLayer("Name","actorReLU2")
    fullyConnectedLayer(2, "Name", "actorFC3")
    tanhLayer("Name", "Action")];

criticOpts = rlRepresentationOptions("LearnRate",1e-3,"L2RegularizationFactor",1e-4,"GradientThreshold",1);
critic = rlQValueRepresentation(criticNetwork,stateInfo,actInfo,"Observation",{'State'},"Action",{'Action'},criticOpts);


actorOptions = rlRepresentationOptions("LearnRate",1e-4,"L2RegularizationFactor",1e-4,"GradientThreshold",1);
actor = rlDeterministicActorRepresentation(actorNetwork,stateInfo,actInfo,"Observation",{'State'},"Action",{'Action'},actorOptions);
