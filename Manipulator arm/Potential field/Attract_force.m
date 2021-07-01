function F = Attract_force(p,pf,z,d)
% ATTRACT_FORCE creates the conic well potential field and the parabolic 
% well field and calculates the attractive force for every joint on the
% robot.
%  
%
% INPUTS:
%   p  - the matrix which contains the current positions of each joint
%   pf - the matrix which contains the final positions of each joint
%   z  - the parameter of the attractive field strength (force per distance)
%   d  - the distance where conic changes to parabolic (mm)
%
% OUTPUTS:
%   F   - a matrix which contains the attractive force acted on each joint


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

a = z*d;                     %The strength of the conic well field
dis = norm(p(:,7)-pf(:,7));  %Calculate the distance to the goal in workspace

%Calculate virtual forces on joint 2,3,4,5,6 and the end effector:
F = [];                      %initialize the force matrix

% Determine which field to use
if (dis > d)   
    for i = 1:6
        %Attractive forces in conic well potential field:
        if norm(p(:,i+1)-pf(:,i+1)) == 0  
            Fi = [0; 0; 0];
        else
            Fi = -a*(p(:,i+1)-pf(:,i+1))/norm(p(:,i+1)-pf(:,i+1));
        end
        F = [F Fi];          %Concate two force matrices together
    end
else
    for i = 1:6
        %Attractive forces in parabolic well potential field:
        Fi = -z*(p(:,i+1)-pf(:,i+1));
        F = [F Fi];          %Concate two force matrices together
    end
end


end