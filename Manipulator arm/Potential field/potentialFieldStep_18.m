function [qNext, isDone] = potentialFieldStep_18(qCurr, map, robot)
% POTENTIALFIELDSTEP_18 Calculates a single step in a potential field
%   planner based on the virtual forces exerted by all of the elements in
%   map. This function will be called over and over until isDone is set.
%   Use persistent variables if you need historical information. CHANGE 
%   GROUPNO TO YOUR GROUP NUMBER.
%
% INPUTS:
%   qCurr - 1x6 vector representing the current configuration of the robot.
%   map   - a map struct containing the boundaries of the map, any
%           obstacles, the start position, and the goal position.
%   robot - a struct of robot parameters
%
% OUTPUTS:
%   qNext  - 1x6 vector representing the next configuration of the robot
%            after it takes a single step along the potential field.
%   isDone - a boolean flag signifying termination of the potential field
%            algorithm. 
%
%               isDone == 1 -> Terminate the planner. We have either
%                              reached the goal or are stuck with no 
%                              way out.
%               isDone == 0 -> Keep going.



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algorithm Starts Here             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
isDone = 0;       %Initialize the value of isDone to 0

%% 
%Set parameters:
alpha = 0.02;     %The fixed step size in joint space
d_judge = 50;     %The distance where distance check takes effect (mm)

distance = 50;    %The distance where conic changes to parabolic (mm)
zeta = 0.01;      %The strength of the parabolic attractive field
eta = 20000;      %The strength of the repulsive field
r0 = 400;         %The distance where repulsive forces take effect

localmini = 0.03;  %The local minimum threshold

global qpast      %qpast stores all the past configurations
x = size(qpast);

%Force Calculation:
qgoal = map.goal;
%Find postition matrix by using forward kinematics method:
[pos,trans] = calcFK(qCurr,robot);
[posf,transf] = calcFK(qgoal,robot);

%Calculate attractive forces on each origin:
Fatt = Attract_force(pos,posf,zeta,distance);

%Calculate repulsive forces on each origin:
Frep = Repulse_force(pos,map,eta,r0);

%Sum all joint efforts:
Ftot = Fatt' + Frep; 
Ftot = [Ftot, zeros(6,3)];      %Fill up total force matrix with zeros 

for i = 1:6
    %This generates a 6 x (joint-1) matrix
    Jv = calcJacobian_18(qCurr, i+1, robot); 
    %Find out the joint efforts required to create the attractive force: 
    tau(:,i) = [forceToTorque_18(Ftot(i,:), Jv);zeros(6-i,1)];  
end

tau_tot = sum(tau,2);           %Add the torques together for each joint

%Take a fixed-magnitude step in joint space in direction of joint efforts
%to obtain new joint configurations:
qNext = qCurr + alpha * tau_tot'/norm(tau_tot');  

isCollide = 0;
%Check for collisions:
if (isRobotCollided(qNext, map, robot) == 1)
    isCollide = 1; 
    disp('The next configuration is in collision, try a new q');
end

isStuck = 0;
%Check local minimum:
if (x(1) > 10)
    isStuck = 1;
    for i = 0:9
        if norm(qpast(x(1)-i,:)-qNext) > localmini
            isStuck = 0;
        end
    end
    if (isStuck)  
        disp('The robot is in local minimum, try a new q');
    end
end

%If collide or if stuck in local minimum, try a random new q
while (isStuck || isCollide)
    oldq = qNext;
    randomt = pi*(0.5-rand(1));
    qNext(1) = qNext(1)+randomt; 
    %Test if the new q collides with obstacles
    if (isRobotCollided(qNext, map, robot) == 0) 
        isCollide = 0;
        isStuck = 0;
        %If not collide, move the robot to the new configuration
        %(escape the local minimum)
        for i = 1:10
            lynxServo(oldq+i*(qNext-oldq)/10);
            pause(0.05);
        end
    end   
end    
qpast = [qpast;qNext];

%Check distance to the goal:
if isDone == 0
    [posNext, transNext] = calcFK(qNext, robot);
    d_tot = 0;                  %Initialize the d_tot
    
    %Calculate distance between current position and goal position and add
    %them together:
    for i = 2:7
        dist(i-1) = sqrt((posf(1, i) - posNext(1,i))^2 +...
            (posf(2, i) - posNext(2,i))^2 + (posf(3, i) - posNext(3,i))^2);
        d_tot = d_tot + dist(i-1);  
    end 
    %if d_tot is less than d_judge, then terminate the planar:
    if d_tot < d_judge 
        isDone = 1;
    end 
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algorithm Ends Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end