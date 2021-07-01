function tau = forceToTorque_18(F, J)
% FORCETOTORQUE_18 Calculate the joint torques required to exert a 
%   specific set of end-effector forces/torques in a given configuration.
%
% INPUTS:
%   F - 1 x 6 vector of desired forces/torques (where F(1:3) is the
%       forces and F(4:6) is the torques)
%   J - the 6xN jacobian of the robot in its current configuration  
%
% OUTPUTS:
%   tau - Nx1 vector of joint torques required to generate F.

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Your Code Starts Here             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Jvt = transpose(J);         % Calculate the transposed Jacobian matrix
tau = Jvt * transpose(F);   % Use the Jacobian matrix to convert force to torque

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Your Code Ends Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end