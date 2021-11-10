% Used for Motion Planning for Mobile Robots
% Thanks to HKUST ELEC 5660 
close all; clear all; clc;
addpath('A_star')

% Environment map in 2D space 
xStart = 1.0;
yStart = 1.0;
xTarget = 300.0;
yTarget = 300.0;
MAX_X = 300;
MAX_Y = 300;
 map = obstacle_map(xStart, yStart, xTarget, yTarget, MAX_X, MAX_Y);
%map = [1 1;4 1;4 2;4 3; 4 4; 4 5;4 7; 4 9; 4 8; 7 1; 7 2; 7 4; 7 5;7 6;7 7;7 8;7 9;7 10;7 11;4 10;4 11;8 8; 15 15];
% Waypoint Generator Using the A* 
[path,visit_node] = A_star_search(map, MAX_X,MAX_Y);

% visualize the 2D grid map
visualize_map(map, path,visit_node);

% save map
% save('Data/map.mat', 'map', 'MAX_X', 'MAX_Y');
