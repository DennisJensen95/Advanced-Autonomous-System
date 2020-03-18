clear
clc
close all

%% Construct map and init
global map
global queue
start = [9,9];
goal = [2,2];
queue = {};

map = zeros(10,10);
map(1:7,3) = ones(7,1);
map(3:10) = ones(8,1);
map(3:10,6) = ones(8,1);
map(goal(1),goal(2)) = 2; %goal
map(start(1),start(2)) = -1; %start

PrintMap()

%% Main

grid_size = 0.05;

Makewave(goal, start);
[route,mission] = FindRoute(start, grid_size);
PrintRoute(route);

CreateSMR(mission);


%% Functions
function CreateSMR(mission)

    x = mission(:,1);
    y = mission(:,2);
    th = mission(:,3);

    fid = fopen( 'ex12-problem6', 'wt' );
    for i = 1:length(mission)
        fprintf(fid, 'drive %.2f %.2f %.2f : ($targetdist<0.01)\n', x(i), y(i), th(i));
    end
    fclose(fid);
end

function PrintRoute(route)
    global map
    [m,n] = size(route);
    
    fprintf('Route:\n')
    for i = 1:m
        fprintf('* ');
        for j = 1:n
            if map(i,j) == 1
                fprintf('* ');
            elseif map(i,j) == 2
                fprintf('G ');
            elseif route(i,j) == 0
                fprintf('0 ');
            elseif route(i,j) == 1
                fprintf('\\ ');
            elseif route(i,j) == 2
                fprintf('| ');
            elseif route(i,j) == 3
                fprintf('/ ');
            elseif route(i,j) == 4
                fprintf('- ');
            end
        end
        fprintf('*\n');
    end
end

function [routeMap, mission] = FindRoute(start,grid_size)
%FINDROUTE FUNCTION
    global map
    
    routeMap = zeros(size(map));
    mission = [];
    
    cp = start;
    while true
        nbs = FindNeighbours(cp);
        [dx, val] = FindMin(nbs,cp);
        routeMap = AddMapSyntax(routeMap,dx,cp);
        
        mission = [mission; FindPose(dx,cp,grid_size)];
        
        if val == 2
            break;
        end
        
        cp = cp+dx;
    end
end

function pose = FindPose(dx,cp,grid_size)
    global map
    
    point = cp+dx;
    
    pose = zeros(1,3);
    
    [m,n]=size(map);
    [x,y]=meshgrid(0:grid_size:((m-1)*grid_size),0:grid_size:((n-1)*grid_size));
    x=x+0.025;
    y=flipud(y)+0.025;
    
    pose(1) = x(point(1),point(2));
    pose(2) = y(point(1),point(2));
    pose(3) = atan2(-dx(1),dx(2))*180/pi;
    
    
end

function map = AddMapSyntax(map, dx, point)
    if isequal(dx,[-1,-1]) || isequal(dx,[-1,-1]) %left-up + right-down
        map(point(1),point(2)) = 1;%'\';
    elseif isequal(dx,[-1,0]) || isequal(dx,[1,0]) %up + down
        map(point(1),point(2)) = 2;%'|';
    elseif isequal(dx,[-1, 1]) || isequal(dx,[1, -1]) %right-up + left-down
        map(point(1),point(2)) = 3;%'/';
    elseif isequal(dx,[0, 1]) || isequal(dx,[0, -1]) %right + left
        map(point(1),point(2)) = 4;%'-';
    end
    
end

function [dx, val] = FindMin(nbs,start)
    global map
    
    val = 1e6;
    
    for i = 1:length(nbs)
        nb = nbs{i};
        
        if map(nb(1),nb(2)) < val && map(nb(1),nb(2)) > 1
            val = map(nb(1),nb(2));
            dx = nb(1:2) - start;
        end
    end

end

function Makewave(goal, start)
% MAKEWAVE FUNCTION
% Create distance map
% Requires predefined global variables 'map' and 'queue'
% INPUT: 1x2 vector of goal position
%        1x2 vector of start position

    global map
    global queue
    queue = {goal};
    
    N = length(queue);
    i = 1;
    while i <= N
        cp = ExtractPoint(i);
        
        if isequal(cp,start)
            break;
        end
        neighbours = FindNeighbours(cp);
        
        for j = 1:length(neighbours)
            nb = neighbours{j};
            d = map(cp(1),cp(2)) + DistanceNeighbour(nb);
            if map(nb(1),nb(2)) == 0
                map(nb(1),nb(2)) = d;
                InsertPoint(nb);
            else
                if (map(nb(1),nb(2)) > d)
                    map(nb(1),nb(2)) = d;
                end
            end
        end
        
        i = i + 1;
        N = length(queue);
    end
end

function dist = DistanceNeighbour(point)
    if length(point) > 2
        dist = 1.41;
    else
        dist = 1;
    end
end

function neighbours = FindNeighbours(point)
% FINDNEIGHBOURS function
% This function calculates all the neighbours of a point in a map
% OUTPUT: 1xN cell with all N neighbours in map
% INPUT: 1x2 vector with point in map
    global map
    neighbours = {};
    
    [m,n] = size(map);
    
    %above
    nb = point + [-1, 0];
    if ~(nb(1) > m || nb(1) < 1 || nb(2) > n || nb(2) < 1)
        neighbours = {neighbours{:}, nb};
    end
    
    %right-up
    nb = point + [-1, 1];
    nb = [nb, 0];
    if ~(nb(1) > m || nb(1) < 1 || nb(2) > n || nb(2) < 1)
        neighbours = {neighbours{:}, nb};
    end
    
    %right
    nb = point + [0, 1];
    if ~(nb(1) > m || nb(1) < 1 || nb(2) > n || nb(2) < 1)
        neighbours = {neighbours{:}, nb};
    end
    
    %right-down
    nb = point + [1, 1];
    nb = [nb, 0];
    if ~(nb(1) > m || nb(1) < 1 || nb(2) > n || nb(2) < 1)
        neighbours = {neighbours{:}, nb};
    end
    
    %down
    nb = point + [1, 0];
    if ~(nb(1) > m || nb(1) < 1 || nb(2) > n || nb(2) < 1)
        neighbours = {neighbours{:}, nb};
    end
    
    %left-down
    nb = point + [1, -1];
    nb = [nb, 0];
    if ~(nb(1) > m || nb(1) < 1 || nb(2) > n || nb(2) < 1)
        neighbours = {neighbours{:}, nb};
    end
    
    %left
    nb = point + [0, -1];
    if ~(nb(1) > m || nb(1) < 1 || nb(2) > n || nb(2) < 1)
        neighbours = {neighbours{:}, nb};
    end
    
    %left-up
    nb = point + [-1, -1];
    nb = [nb, 0];
    if ~(nb(1) > m || nb(1) < 1 || nb(2) > n || nb(2) < 1)
        neighbours = {neighbours{:}, nb};
    end
    
end
    

function point = ExtractPoint(idx)
    global queue
    point = queue{idx};
end

function InsertPoint(point)
    global queue
    queue = {queue{:}, point(1:2)};
end


function PrintMap()
    global map
    [m,n] = size(map);
    
    fprintf('Map:\n')
    for i = 1:m
        fprintf('* ');
        for j = 1:n
            if map(i,j) == 0
                fprintf('0 ');
            elseif map(i,j) == 1
                fprintf('* ');
            elseif map(i,j) == 2
                fprintf('G ');
            else
                fprintf('S ');
            end
        end
        fprintf('*\n');
    end
    fprintf('\nGoal: G\t\t\tStart: S\n\n\n');
    
    map(map == -1) = 0;
end
