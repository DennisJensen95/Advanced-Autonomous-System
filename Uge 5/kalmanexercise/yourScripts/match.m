function [ matchResult ] = match( pose, poseCov, worldLines, laserLines )
% [matchResult] = MATCH(pose,poseCov,worldLines,laserLines)
%   This function matches the predicted lines to the extracted lines. The
%   criterion for a match is the mahalanobis distance between the (alpha,
%   r) parameters of the predicted and the extracted lines. The arguments
%   are:
%       pose: The estimated robot pose given as [x,y,theta]
%       poseCov: The estimated covariance matrix of the robot pose
%       worldLines: Known world lines in world coordinates, given as
%       [alpha;r] for each line. Number of rows = number of lines
%       laserLines: Lines extracted from the laser scan. Given as [alpha;r]
%       for each line. Number of rows = number of lines
%
%       matchResult: A (5xnoOfWorldLines) matrix whose columns are 
%       individual pairs of line matches. It is structured as follows:
%       matchResult = [ worldLine(1,1) , worldLine(1,2) ...  ]
%                     [ worldLine(2,1) , worldLine(2,2)      ]
%                     [ innovation1(1) , innovation2(1)      ]
%                     [ innovation1(2) , innovation2(2)      ]
%                     [ matchIndex1    , matchIndex2    ...  ]
%           Note that the worldLines are in the world coordinates!

    % The varAlpha and varR are the assumed variances of the parameters of
    % the extracted lines, they are read globally.
    global varAlpha varR
    
    noOfWorldLines = length(worldLines);
    [~,noOfLaserLines] = size(laserLines);
    
    matchResult = zeros(5,noOfWorldLines);
    projectedLines = zeros(2,noOfWorldLines);
    lineCov = zeros(2,2,noOfWorldLines);
    g=2;
    
    matchResult(1:2,:) = worldLines;
    
    sigRJ = [varAlpha,0;0,varR];

    for i = 1:noOfWorldLines
        [ projectedLines(1:2,i), lineCov(:,:,i) ] = projectToLaser(worldLines(:,i),pose,poseCov);
        lineCov(:,:,i) = lineCov(:,:,i)+sigRJ;
    end
    
    for i = 1:noOfWorldLines
        for j = 1:noOfLaserLines
            innovation = laserLines(:,j) - projectedLines(:,i);
            
            matching = (innovation.')*(lineCov(:,:,i)\innovation) <= g^2;
            
            if matching == 1
                matchResult(3:4,i) = innovation;
                matchResult(5,i) = j;
                break;
            end
        end
    end
end
