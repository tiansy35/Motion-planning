function [pos,T] = calcFK(q,robot)
%CALCFK utilizes the forward kinematic method to calculate the T-matrix and
%find out the postions of each joint of the Lynx robobt
%
% INPUTS:
%   q       - 1x6 vector representing the configuration of the robot
%   robot   - a struct of robot parameters
% OUTPUTS:
%   pos     - a matrix which contains postions of each joint of the robot
%   T       - Transformation matrix generated from DH convention 
% 


%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
pos = [];       %Initialize pos matrix
T = [];         %Initialize pos matrix

%Set variables: Retrive robot parameters from 'robot.mat' file
d1 = robot.d1;  
a2 = robot.a2;
a3 = robot.a3;
d4 = robot.d4;
d5 = robot.d5;
d6 = robot.d6;

%Retrive joint variables from the configuration q
t1 = q(1);t2 = q(2);t3 = q(3);t4 = q(4);t5 = q(5);t6 = q(6);

%Calculate FK using DH Convention
A1 = DH(0, -pi/2, d1, t1);
A2 = DH(a2, 0, 0, t2-pi/2);
A3 = DH(a3, 0, 0, t3+pi/2);
A4 = DH(0, -pi/2, 0, t4-pi/2);
A5 = DH(0, pi/2, d5, t5+pi/2);
A6 = DH(0, 0, d6, t6);

%Frame 4's origin is not on joint 5, create a new matrix to calculate the 
%joint's position
A4_joint = DH(d4, 0, 0, t4);  

%T-matrices for different link
T1 = A1;
T2 = A1*A2;
T3 = A1*A2*A3;
T4 = A1*A2*A3*A4;
T5 = A1*A2*A3*A4*A5;
T6 = A1*A2*A3*A4*A5*A6;
T4_joint = A1*A2*A3*A4_joint; 
%Concate the T-matrices so that it is easier for other functions to use
T = [T1; T2; T3; T4; T5; T6; T4_joint];

pos(:,1) = [0; 0; 0];       %Set the position of the first joint 

%Take joints' positions from Trans
for i = 1:7                 
    startrow = 1+4*(i-1);   %Define startrow
    endrow = startrow+2;    %Define endrow
    pos(:,i+1) = T(startrow:endrow,4);  %Fill T-matrix information into pos matrix
end

end