%% Setup

close all
addpath('utils')
addpath('maps')

%% Simulation Parameters
%
% Define any additional parameters you may need here. Add the parameters to
% the robot and map structures to pass them to astar or rrt.
%
load 'robot.mat' robot

start = [0,0,0,0,0,0];
goal = [1,0,0,0,0,0];

map = loadmap('maps/map2.txt');

%% Run the simulation

% Solve the path problem using A*
tic
% OR Solve the path problem using RRT
[path] = rrt(map,start,goal)
toc

%% Plot the output

% plotLynxPath(map,path,10);
