
clc;
clear;
close all;
map_path = '/home/colibri/clbri_ws/src/colibri_pathfinding/maps/626_mdf.pgm';
data = 254 - imread(map_path);
logic_data = logical(data);
map_self = robotics.BinaryOccupancyGrid(logic_data, 1);
show(map_self);
node_path = '/home/colibri/clbri_ws/src/colibri_pathfinding/path/nodes.txt';
path_node = load(node_path);

hold on;
plot(path_node(:,1)', 307 - path_node(:,2)', '*');

node_x = path_node(:,1)';
node_y = 307-path_node(:,2)';
smooth_x = mean5_3( node_x, 2 );
smooth_y = mean5_3( node_y, 2 );

figure(2);
show(map_self);
hold on;
plot(smooth_x, smooth_y, '*');
