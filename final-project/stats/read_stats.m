clear
clc
close all
%% select files

choice = 1; % choose the pair of files you want to compare

% set path to folder with files

pathToFolder = strcat(pwd, '/raw');
direc = dir(pathToFolder);

% files in directory (omits "." and "..")
[numOfFiles, ~] = size(direc);
numOfFiles = numOfFiles - 2;
direc = direc(3:end);
files_2 = direc(1:numOfFiles/2);
files_1 = direc(numOfFiles/2+1:end);

% files_1 = ['raw/', direc(choice+numOfFiles/2).name];
% files_2 = ['raw/', direc(choice).name];

%% compute similarity
correctObjDetect = zeros(2, 4);
totalResultMatrix = zeros(1,4);

for k = 1:numOfFiles/2
    file1 = ['raw/', files_1(k).name];
    file2 = ['raw/', files_2(k).name];
    
    experiment = create_python_table(file1);
    test = table2array(readtable(file2));
    
    [measurements, ~] = size(test);
    [experiments, ~] = size(experiment);
    fprintf('Number of experiments NOT completed: %d\n\n', experiments-measurements);

    % matrix to store result: 0 = wrong, 1 = correct
    resultMatrix = zeros(measurements,4);
       
    for i =1:experiments
        object = experiment(i, 1);
        correctObjDetect(2, object) = correctObjDetect(2, object) + 1;
    end

    skip = 0;
    for i = 1:measurements
        %extract measurement data
        object = test(i,1);
        objx = test(i,2);
        objy = test(i,3);
        objPose = test(i,4);
        
        % skip if no object found
        if object == 0
            continue
        end
        j = i+skip;  % set iteration variable
        threshold_xy = 0.1;  % set threshold value
        while j < experiments
            skipFlag = true;
            % the main check is to see if the x,y position is correct
            if abs(objx - experiment(j,2)) < threshold_xy && ...
                    abs(objy - experiment(j,3)) < threshold_xy && object == experiment(j,1)
                
                correctObjDetect(1, object) = correctObjDetect(1, object) + 1;
                resultMatrix(i,1:3) = 1;

                % check if correct pose
                if abs(objPose - experiment(j,4)) < 0.1
                    resultMatrix(i,4) = 1;
                elseif (experiment(j,1) == 1 || experiment(j,1) == 2) 
                    temp = abs(experiment(j,4) - objPose);

                    % in the case of a square, check if measurement is 180
                    % degrees apart
                    if temp < 0.05 || abs(pi-temp) < 0.05
                        resultMatrix(i,4) = 1;
                    end
                end
                break;
            end

            % if increased iterations, the threshold decreases
            if j == i + skip + 3
                threshold_xy = 0.05;
            elseif j == i + skip + 6
                threshold_xy = 0.01;
            end

            j = j + 1;

            if j > i+10
                skipFlag = false;
                break;
            end
        end

        if skipFlag
            skip = j - i;
        end
    end

    totalResultMatrix = [resultMatrix; totalResultMatrix];

end

%% display result

disp(correctObjDetect)

% fully correct
allCorrect = sum(sum(totalResultMatrix,2)==4);

% correct object
objCorrect = sum(totalResultMatrix(:,1) == 1);

% correct x and y
xyCorrect = sum(sum(totalResultMatrix(:,2:3),2) == 2);

% correct pose
poseCorrect = sum(totalResultMatrix(:,4) == 1);

measurements = size(totalResultMatrix, 1) - 1;
fprintf('Out of %d completed experiments...\n', measurements);
fprintf('\t%d (%.2f %%) was fully correct\n', allCorrect, allCorrect/measurements*100);
fprintf('\t%d found the correct object\n', objCorrect);
fprintf('\t%d found the correct position\n', xyCorrect);
fprintf('\t%d found the correct pose\n', poseCorrect);


fprintf('For object 1 the accuracy was %.2f %%\n', correctObjDetect(1,1)/correctObjDetect(2,1)*100)
fprintf('For object 2 the accuracy was %.2f %%\n', correctObjDetect(1,2)/correctObjDetect(2,2)*100)
fprintf('For object 3 the accuracy was %.2f %%\n', correctObjDetect(1,3)/correctObjDetect(2,3)*100)
fprintf('For object 4 the accuracy was %.2f %%\n', correctObjDetect(1,4)/correctObjDetect(2,4)*100)
%% display actual result

x = [1 2 3, 4];
vals = correctObjDetect(1,:)./correctObjDetect(2,:)*100;
grid
b = bar(x,vals);
ylim([0, 105]);
xtips1 = b.XData;
ytips1 = round(b.YData, 1);
labels1 = strcat(string(ytips1), "%");
xlabel('Object number')
ylabel('Accuracy score %')
text(xtips1,ytips1,labels1,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom')

%%
function A = create_python_table(filename)
    py = readtable(filename);
    [m, ~] = size(py);
    
    A = zeros(m,4);
    
    A(:,1) = py.Var1;
    A(:,4) = py.Var3;
    
%     py.Var2{1};

    new = strrep(py.Var2, ',', '');
    new = strrep(new,'[', '');
    new = strrep(new,']', '');

    for i = 1:m
        numstr = split(new{i});
        A(i,2) = str2double(numstr{1});
        A(i,3) = str2double(numstr{2});
    end
end
