function [A]=DH(a,alpha,d,theta)
% DH: calculate the matrix A for the corresponding link using 
%     DH parameters
%
% INPUT: 
% a      - Link Length: distance between zi-1 and zi, measured 
%          along xi
% alpha  - Link Twist: angle between zi-1and zi,measured in the 
%          plane normal to xi(right hand rule)
% d      - Link Offset: distance between xi-1and xi,measured 
%          along zi-1
% theta  - Joint Angle: angle between xi-1and xi,measured in the 
%          plane normal to zi-1(right hand rule)
%
% OUTPUT: 
% A      - The transformation matrix between each pair of 
%          frames
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Code Here:
Tranx = [1 0 0 a;... %Homogeneous transfermations: move along x
           0 1 0 0;...
           0 0 1 0;...
           0 0 0 1];
Tranz = [1 0 0 0;... %Homogeneous transfermations: move along z
           0 1 0 0;...
           0 0 1 d;...
           0 0 0 1];
Rotx = [1 0 0 0;... %Homogeneous transfermations: rotate about x
      0 cos(alpha) -sin(alpha) 0;...
      0 sin(alpha) cos(alpha) 0;...
      0 0 0 1];
Rotz = [cos(theta) -sin(theta) 0 0;... %Homogeneous transfermations: rotate about z
      sin(theta) cos(theta) 0 0;...
      0 0 1 0;...
      0 0 0 1];
A=Rotz*Tranz*Tranx*Rotx; 

end

