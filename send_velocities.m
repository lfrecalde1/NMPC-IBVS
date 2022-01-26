function  send_velocities(robot, velmsg, vd, flag)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% Send desired velocities to the robot Linear
vx = vd(1);
vy = vd(2);
vz = vd(3);

% Send desired velocities to the robot angular
wx = vd(4);
wy = vd(5);
wz = vd(6);

if flag ==1
    velmsg.Linear.X = vx;
    velmsg.Linear.Y = vy;
    velmsg.Linear.Z = vz;
    
    velmsg.Angular.X = wx;
    velmsg.Angular.Y = wy;
    velmsg.Angular.Z = wz;
else
    velmsg.Linear.X = 0;
    velmsg.Linear.Y = 0;
    velmsg.Linear.Z = 0;
    
    velmsg.Angular.X = 0;
    velmsg.Angular.Y = 0;
    velmsg.Angular.Z = 0;
end


send(robot,velmsg);
end

