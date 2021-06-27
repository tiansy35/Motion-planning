function connected = connectstart(q,i,map,robot)
%CONNECTSTART finds paths between the current configuration to the start
%tree, checks collision on the paths and stores the shortest feasible path.
%
% INPUT:
%   q     - the configuration of the current configuration
%   i     - the index of the configration q
%   map   - map specifications
%   robot - robot specifications
%
% OUTPUT:
%   connected - judgment flag for the function 
%


global pathstart; %define a global variable

for j = 1:i-1            %calculate the length of possible paths
    distance(j) = sqrt((q(i,1)-q(j,1))^2+(q(i,2)-q(j,2))^2+(q(i,3)-q(j,3))^2+(q(i,4)-q(j,4))^2);
end

%find the shortest path to the previous points
[mindis,index] = min(distance);  
%set qstart to be the start point of the line segment of the path 
qstart = q(index,:); 
%set qsend to be the end point of the line segment of the path
qend = q(i,:);   

flag = 1;
%do linear interpolation on the line and check collision for the middle points
for k = 1:19   
    qmiddle = qstart+(qend-qstart)*k/20;
    if (isRobotCollided(qmiddle,map,robot) == 1) 
        flag = 0;
    end
end
%if the path does not collide with the obstacle, add it to the start tree
if (flag == 1)  
    pathstart{i} = [pathstart{index} i];
end
connected = flag;

end