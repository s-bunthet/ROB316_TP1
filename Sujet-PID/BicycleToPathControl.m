
function [ u ] = BicycleToPathControl( xTrue, Path )
%Computes a control to follow a path for bicycle
%   xTrue is the robot current pose : [ x y theta ]'
%   Path is set of points defining the path : [ x1 x2 ... ;
%                                               y1 y2 ...]
%   u is the control : [v phi]'

% TODO
global goalWaypointId
goalWaypointId = min(goalWaypointId, size(path,2));

rho = 0.5;
xGoal = Path(:,goalWaypointId);
error = xGoal-xTrue;
waypointDist = norm(error(1:2));

if waypointDist < rho
    xGoal = Path(:,goalWaypointId);
    goalWaypointId = goalWaypointId+1;
    goalWaypointId = min(goalWaypointId, size(path,2));
    
    
else
    delta = Path(:,goalWaypointId)-Path(:, goalWaypointId-1);
    delta = delta/norm(delta);
    error = xGoal - xTrue;
    goalDist = norm(error(1:2));
    while goalDist < rho 
      xGoal = xGoal + 0.01*delta;
      error = xGoal - xTrue;
      goalDist = norm(error(1:2));
    end
end

% once we know the next point, we use the controlor to reach that point
Krho = 10;
Kalpha = 5;
error = xGoal-xTrue;
alpha = AngleWrap(atan2(error(2),error(1)))-xTrue(3);

u(1) = Krho*rho;
u(2) = Kalpha*alpha;
end


