function J = calcJacobian_18(q, joint, robot)
% CALCJACOBIAN_18 Calculate the Jacobian of a particular joint of the 
%   robot in a given configuration. CHANGE GROUPNO TO YOUR GROUP NUMBER.
%
% INPUTS:
%   q     - 1 x 6 vector of joint inputs [q1,q2,q3,q4,q5,q6]
%   joint - scalar in [1,7] representing which joint we care about
%   robot - a struct of robot parameters
%
% OUTPUTS:
%   J - 6 x (joint-1) matrix representing the Jacobian
%

%%

J = [];             

if nargin < 3
    return
elseif joint <= 1 || joint > 7
    return
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Your Code Starts Here             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

J = zeros(6,joint-1);           %Initialize J at the beginning

%In this function, Geometric method for calculating Jv:
Jv = [];                        %Initialize linear Jacobian matrix
zaxis = [0; 0; 1];              %Define the axis of z
z(:,1) = zaxis;             
[pos,Trans] = calcFK(q,robot);  %Use forward kinematics to calculate positions of each joints

%Take Z vectors out from Trans matrix
for i = 1:6                     
    startrow = 1+4*(i-1);                   %Define startrow
    endrow = startrow+2;                    %Define endrow
    Rot = Trans(startrow:endrow,1:3);       %Retrive Rot matix from Trans matrix
    z(:,i+1) = Trans(startrow:endrow,3);    %Retrive z vectors out from Trans matrix
end
if (joint == 5) 
    endpos = pos(:,8);          %Special Case: Joint 5 position is at the last column of position matrix
                                %It corresponds to the last column of Trans matrix 
else
    endpos = pos(:,joint);      
end

for i = 1:joint-1               
    Si = S(z(:,i));
    Ji = Si*(endpos-pos(:,i));  %Use S function to achieve cross product
    Jv = [Jv Ji];               %Concate Ji matrix to linear velocity Jacobian matrix
end

%Calculate Jw and combine with Jv
Jw = z(:,1:joint-1);            %Calculate angular velocity Jacobian matrix
J = [Jv; Jw];                   %Concate angular velocity Jacobian and linear velocity Jacobian together



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Your Code Ends Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end