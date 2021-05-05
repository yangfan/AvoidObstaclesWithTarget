function PlotAvoidObstaclesPose(u, mapMatrix, mapScale, range, scanAngles, ax)
%POSEPLOT Summary of this function goes here
%   Detailed explanation goes here
cla(ax);
show(binaryOccupancyMap(mapMatrix, mapScale), "Parent", ax);
hold(ax, "on");
% title('step#: 1000, epsd reward: 530')
plotTransforms([u(1), u(2), 0], eul2quat([u(3), 0, 0]), "MeshFilePath", "groundvehicle.stl", "View", "2D", "Parent", ax);
scan = lidarScan(range, scanAngles);
scan = transformScan(scan, u);
plot(ax, scan.Cartesian(:, 1), scan.Cartesian(:, 2), 'rX');
plotTransforms([7.0, 4.5, 0], eul2quat([-pi/2, 0, 0]), "MeshFilePath","multirotor.stl", "View", "2D");
hold(ax, "off");
drawnow;