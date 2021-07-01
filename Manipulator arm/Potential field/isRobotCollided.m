function [isCollided] = isRobotCollided(q, map, robot)
% ISROBOTCOLLIDED Detect if a configuration of the Lynx is in collision
%   with any obstacles on the map.
%
% INPUTS:
%   q   - a 1x6 configuration of the robot
%   map - a map strucutre containing axis-aligned-boundary-box obstacles
%   robot - a structure containing the robot dimensions and joint limits
%
% OUTPUTS:
%   isCollided - a boolean flag: 1 if the robot is in collision, 0 if not.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algortihm Starts Here             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[m,n] = size(map.obstacles);
for i = 1:m        %expand the obstacle volume for every obstacle
    obstacle(i,1:3) = map.obstacles(i,1:3);   
    obstacle(i,4:6) = map.obstacles(i,4:6);
end
%transfer the configuration to the joint positions in workspace
jointposition = calculateFK_sol(q);   

flag = 0;
for j = 1:m
    for i = 1:6
    %use the given detectCollision function and the joint positions to detect collision, if collide,flag = 1 
        if (detectCollision(jointposition(i,:),jointposition(i+1,:),obstacle(j,:)) == 1)  
        flag = 1;  %Set flag to 1
        end
    end
end

isCollided = flag; %Set the value of flag to isCollided

    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algortihm Ends Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end