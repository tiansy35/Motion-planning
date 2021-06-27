function [path] = rrt(map, start, goal)
% RRT Find the shortest path from start to goal.
%   PATH = rrt(map, start, goal) returns an mx6 matrix, where each row
%   consists of the configuration of the Lynx at a point on the path. The 
%   first row is start and the last row is goal. If no path is found, PATH 
%   is a 0x6 matrix. 
%
% INPUTS:
%   map     - the map object to plan in
%   start   - 1x6 vector of the starting configuration
%   goal    - 1x6 vector of the goal configuration


%% Prep Code

load('robot.mat'); %load robot specifications

%pathstart stores all the possible path numbers in the map
global pathstart;   
pathstart = {};
q = [];
path = [];  %path is the final and shortest path that are chosen
flag = 0;
flagstart = 0;
flaggoal = 0;
i = 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algorithm Starts Here             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


pathstart = {[1]};
q(i,:) = [start];     %the first configuration is the start point
i = i+1;
while (flag == 0)
    q(i,:) = sample_q(q(i-1,:));   %sample a new configuration
    qcurrent = q(i,:);
    %check collision of the new sample q
    if (isRobotCollided(qcurrent,map,robot) == 0) 
        %connect the new point to previous points
        flagstart = connectstart(q,i,map,robot); 
        %connect the new point to the goal
        flaggoal = connectgoal(q(i,:),goal,map,robot);    
        if (flagstart == 1)
            %if one point is connected with both the tree from the start and the goal, break the loop
            if (flaggoal == 1)  
                flag = 1; 
            end
            i = i+1;
        end
    end
end

%select the final path which connects both start and goal
i = i-1;
pathnumber = pathstart{i};    
%according to the path number, store the value of configuration along the path
for j = 1:numel(pathnumber)
    path = [path; q(pathnumber(j),:)];  
end
path = [path;goal];  %add the goal as the last configuration


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algorithm Ends Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


end