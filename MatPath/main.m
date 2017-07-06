
clc;
clear;
map_path = '/home/colibri/clbri_ws/src/colibri_pathfinding/maps/Dialate_Img.pgm';
data = 254 - imread(map_path);
logic_data = logical(data);
map_self = robotics.BinaryOccupancyGrid(logic_data, 1);
show(map_self);
node_path = '/home/colibri/clbri_ws/src/colibri_pathfinding/path/nodes.txt';
path_node = load(node_path);

hold on;
plot(path_node(:,1)', 307-path_node(:,2)', '*');
