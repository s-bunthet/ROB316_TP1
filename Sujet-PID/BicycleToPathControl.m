
function [ u ] = BicycleToPathControl( xTrue, Path )
%Computes a control to follow a path for bicycle
%   xTrue is the robot current pose : [ x y theta ]'
%   Path is set of points defining the path : [ x1 x2 ... ;
%                                               y1 y2 ...]
%   u is the control : [v phi]'

% TODO
global path_id
path_id = min(path_id, size(Path,2));

rho        = 0.4; %chosen value (trial and error...)
xGoal      = Path(:,path_id); %current goal
error      = xGoal - xTrue;
distToGoal = norm(error(1:2));

if distToGoal < rho
    path_id = path_id+1;
    path_id = min(path_id, size(Path,2));
    xGoal   = Path(:,path_id); 
else
    delta      = Path(:,path_id) - Path(:, path_id-1);
    delta      = delta/norm(delta);
    error      = xGoal - xTrue;
    distToGoal = norm(error(1:2));
    while distToGoal > 1.1*rho 
      xGoal      = xGoal - 0.01*delta;
      error      = xGoal - xTrue;
      distToGoal = norm(error(1:2));
    end
end

% once we know the next point, we use the controlor to reach that point
Krho   = 25; %almost the same as in BicycleToPointControl
Kalpha = 10; %the same as in BicycleToPointControl
error  = xGoal - xTrue;
alpha  = AngleWrap(atan2(error(2),error(1)) - xTrue(3));

u(1) = Krho*rho;
u(2) = Kalpha*alpha;
end


