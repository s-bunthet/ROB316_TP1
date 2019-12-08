function [ u ] = UnicycleToPoseControl( xTrue,xGoal )
%Computes a control to reach a pose for unicycle
%   xTrue is the robot current pose : [ x y theta ]'
%   xGoal is the goal point
%   u is the control : [v omega]'

% TODO
Krho = 15;
Kalpha = 10;
Kbeta = 20;
alpha_max = 1.57;

error = xGoal-xTrue;
beta = error(3);
rho = norm(error(1:2));
alpha = AngleWrap(atan2(error(2),error(1)))-xTrue(3);

u(1) = Krho*rho;
if abs(alpha)>alpha_max
    u(1) = 0;
end

u(2) = Kalpha*alpha;
if rho < 0.05
    u(2) = Kbeta*beta;
end



end


