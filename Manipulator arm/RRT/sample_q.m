function qsample = sample_q(lastq)
%sample_q Sample a new point in the configuration space with the limits of 
% joint limits and maximum step size. 
%
% INPUT:
%    lastq - the last configuration 
%
% OUTPUT:
%    qsample - new configuration by sampling random points from it

load('robot.mat');

jointlimUpp = robot.upperLim;       %take joint limits into account
jointlimLow = robot.lowerLim;
steplimUpp = lastq+0.6;     %set the maximum step size = 0.6 for each joint
steplimLow = lastq-0.6;

for i = 1:6 
    %combine the limits from the physical joint limit and the maximum step size
    Limitupper(i) = min(jointlimUpp(i),steplimUpp(i));   
    Limitlower(i) = max(jointlimLow(i),steplimLow(i));
    %sample a random q between the lower and upper limits for the first 4 joints
    if (i <= 4)
        qrand(i) = Limitlower(i)+(Limitupper(i)-Limitlower(i))*rand(1);  
        %generate random configration points 
    end
end

qsample = [qrand 0 0];  %assume the last two joint variables are zero

end