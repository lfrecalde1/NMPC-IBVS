function [h,hp] = odometry(odom, L)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Robot constant parameters
a = L(1);
b = L(2);
c = L(3);

% Read odometry values from ros
odomdata = receive(odom,3);
pose = odomdata.Pose.Pose;
vel = odomdata.Twist.Twist;
quat = pose.Orientation;
angles = quat2eul([quat.W quat.X quat.Y quat.Z]);

% Get values of position an orientation
x = pose.Position.X;
y = pose.Position.Y;
z = pose.Position.Z;

yaw = angles(1);
pitch = angles(2);
roll = angles(3);

% Direct Kinematics
x = x +a*cos(yaw) - b*sin(yaw);
y = y +a*sin(yaw) + b*cos(yaw);
z = z + c;

% Get values of linear an angular velocities
vx = vel.Linear.X;
vy = vel.Linear.Y;
vz = vel.Linear.Z;

wx = vel.Angular.X;
wy = vel.Angular.Y;
wz = vel.Angular.Z;

% create vector of position and angular states
h = [x;...
     y;...
     z;...
     roll;...
     pitch;...
     yaw];

% Create vector of linear an angular velocities
hp = [vx;...
      vy;...
      vz;...
      wx;...
      wy;...
      wz];

end

