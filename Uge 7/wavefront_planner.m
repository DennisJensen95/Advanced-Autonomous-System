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
map(goal(1),goal(2)) = 2; %goal
map(start(1),start(2)) = -1; %start

PrintMap()

%% Main

Makewave(goal, start);
route = FindRoute(start);
PrintRoute(route);


%% Functions
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
                fprintf('G');
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

function routeMap = FindRoute(start)
%FINDROUTE FUNCTION
    global map
    
    routeMap = zeros(size(map));
    
    cp = start;
    while true
        nbs = FindNeighbours(cp);
        [dx, val] = FindMin(nbs,cp);
        routeMap = AddMapSyntax(routeMap,dx,cp);
        
        if val == 2
            break;
        end
        
        cp = cp+dx;
    end
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
