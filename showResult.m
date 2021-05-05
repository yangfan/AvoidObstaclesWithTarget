
% out = trainingStats_newtrain7.SimulationInfo(82, 1);
% out = oot;
fig = figure("Name","simpleMap");
set(fig, "Visible", "on");
ax = axes(fig);
show(binaryOccupancyMap(mapMatrix),"Parent",ax);

for i = 1:5:size(out.range, 3)
    u = out.pose(i, :);
    r = out.range(:, :, i);
    PlotAvoidObstaclesPose(u, mapMatrix, mapScale, r, scanAngles, ax);
end
