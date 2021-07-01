function F = Repulse_force(p,map,e,r)
% REPULSE_FORCE creates the repulsive field and calculates the repulsive 
% force between every joint of the robot and each obstacle. 
%
% INPUTS:
%   p   - the matrix which contains the current positions of each joint
%   map - a map struct containing the boundaries of the map, any obstacles, 
%         the start position, and the goal position
%   e   - the parameter of the repulsive field strength (force per distance)
%   r   - the distance of influence of the obstacle (mm)
%
% OUTPUTS:
%   F   - a matrix which contains all repulsive forces acted on each joint 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%

%Pre-settings:
[m,n] = size(map.obstacles);  
p5 = p(:,8);
p(:,5) = p5;
p = p(:,2:7)';

F = zeros(6,3);         %Initialize the F to a 6 by 3 zero matrix 

%Calculate virtual forces on joint 2,3,4,5,6 and the end effector
for i = 1:m
    Fi = [];  
    %Call distPointToBox function to calculate the distance d from a point 
    %to the axis-aligned boundary box:
    [dist,dir] = distPointToBox(p,map.obstacles(i,:));
    for j = 1:6
        %If distance to the obstavle > r, repulsive force = 0
        if (dist(j) > r)  Fj = [0 0 0];
        else
            Fj = (-dir(j,:))*e*(1/dist(j)-1/r)/(dist(j)^2);
        end
        Fi = [Fi;Fj]; %Concate forces on different joints into a larger matrix
    end 
    F = F+Fi;           %Add repulsive forces from different obstacles
end

end