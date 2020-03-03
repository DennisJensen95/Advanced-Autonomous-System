function [ projectedLine, lineCov ] = projectToLaser( worldLine,poseIn, covIn)
%[projectedLine, lineCov] = PROJECTTOLASER(worldLine,poseIn,covIn) 
%Project a word line to the laser scanner frame given the
%world line, the robot pose and robot pose covariance. Note that the laser
%scanner pose in the robot frame is read globally
%   worldLine: The line in world coordinates
%   poseIn: The robot pose
%   covIn: The robot pose covariance
%
%   projectedLine: The line parameters in the laser scanner frame
%   lineCov: The covariance of the line parameters

%% Constants
global lsrRelPose % The laser scanner pose in the robot frame is read globally

%% Line parameters in laser scanner frame

%From world to robot
    aw = worldLine(1);
    rw = worldLine(2);
    xw = poseIn(1);
    yw = poseIn(2);
    tw = poseIn(3);

    ar = aw-tw;
    rr = rw-xw*cos(aw)-yw*sin(aw);

%From robot to laser
    xr = lsrRelPose(1);
    yr = lsrRelPose(2);
    tr = lsrRelPose(3);
    
    al = ar-tr;
    rl = rr-xr*cos(ar)-yr*sin(ar);

    projectedLine = [al;rl];    

%% Covariance of line parameters
lineCov = lineCovp(worldLine,covIn);

function sigmazp=lineCovp(zw,poseCov)
    nh = [0,0,-1;-cos(zw(1)),-sin(zw(1)),0];
    sigmazp = nh*poseCov*(nh.');
end

end
