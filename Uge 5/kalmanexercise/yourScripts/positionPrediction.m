function [ poseOut, covOut ] = positionPrediction( poseIn,covIn,delSr,delSl)
%[poseOut, covOut] = POSITIONPREDICTION(poseIn,covIn,delSr,delSl) perform
%one step of robot pose prediction from a set of wheel displacements
%   poseIn = old robot pose
%   covIn = uncertainty on the old robot pose
%   delSr = right wheel linear displacement
%   delSl = left wheel linear displacement


%% Constants
% The robot parameters are read globally, odoB is the wheel separation, kR
% and kL are the odometry uncertainty parameters
global odoB kR kL 

%% pose update
% poseOut = [0;0;0];

theta = poseIn(3);
poseOut = poseIn + [(delSr+delSl)/2*cos(theta+(delSr-delSl)/(2*odoB));
                    (delSr+delSl)/2*sin(theta+(delSr-delSl)/(2*odoB));
                    (delSr-delSl)/odoB];

%% Covariance update

sigu = [kR*abs(delSr),0;0,kL*abs(delSl)];
delS = (delSr+delSl)/2;
delT = (delSr-delSl)/odoB;


Fp = [[1;0;0],[0;1;0],[-delS*sin(theta+delT/2);
                       delS*cos(theta+delT/2);
                       1]];

a=theta+delT/2;
b=delS/odoB;

FdelRL = [1/2*cos(a)-1/2*b*sin(a),1/2*cos(a)+1/2*b*sin(a);
          1/2*sin(a)+1/2*b*cos(a),1/2*sin(a)-1/2*b*cos(a);
          1/odoB,-1/odoB];

covOut = Fp*covIn*(Fp.') + FdelRL*sigu*(FdelRL.');

end
