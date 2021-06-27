function [flaggoal] = connectgoal(q,goal,map,robot)
% CONNECTGOAL checks collision on the path between the current configuration and the
% goal. It creates multiple different transition configuration q's between q 
% and goal then check each link to see if there is any collision between them 
%
% INPUT:
%   q     - the configuration of the current configuration
%   goal  - the configuration at the goal point
%   map   - map specifications
%   robot - robot specifications
%
% OUTPUT:
%   flaggoal - judgment flag for the function 

flaggoal = 1;  %set flag to 1 and make sure the program enter the if statement

diff = abs(goal-q); %find out the difference between q and goal configurations
for i = 1:4     %only check first 4 joints 
    if diff(i)>0.8
        flaggoal = 0; %abandon the current configration by setting flag to 0
    end
end

if (flaggoal == 1)
    for j = 1:20  %set 20 steps for linear interpolation
        qtrans = q + (j/20)*(goal-q); %apply linear polation for the q configuration
        if isRobotCollided(qtrans, map, robot) == 1
            flaggoal = 0;  %abandon the current configration by setting flag to 0
        end
    end
end



end

        

        

