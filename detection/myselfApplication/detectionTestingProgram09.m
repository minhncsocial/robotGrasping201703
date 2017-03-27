%% Test Fixed Height
%**************************************************************************
%% initialize need directory
clear all; 
% close all; 
clc;

startTime0 = clock;
path_detectionUtils = 'E:\WORK\ORGANIZATION\NTUT\Robot Grasping\Project02\Code\deepGraspingCode1\detection\detectionUtils';
path_utils = 'E:\WORK\ORGANIZATION\NTUT\Robot Grasping\Project02\Code\deepGraspingCode1\util';
temp1 = sprintf(path_detectionUtils,pwd)
temp2 = sprintf(path_utils,pwd);

addpath(path_detectionUtils);
addpath(path_utils);
addpath('myselfUtil');

% addpath ../detectionUtils/
% addpath ../../util

dataDir = 'E:\WORK\ORGANIZATION\NTUT\Robot Grasping\Project02\Code\rawDataSet';
bgrDir = 'E:\WORK\ORGANIZATION\NTUT\Robot Grasping\Project02\Code\rawDataSet';
instNum = sscanf('pcd0110r.png', '%*3c%u')

%% load need data for detection
load ../../data/bgNums.mat
load ../../data/graspModes24.mat
load ../../weights/graspWFinal.mat
load ../../data/graspWhtParams.mat

%% config parameters for detection
% Just use the positive-class weights for grasping.
w_class = w_class(:,1);

bgrFN = sprintf('%s/pcdb%04dr.png',bgrDir,bgNo(instNum));

rotAngs = 0:15:(11*15);
% rotAngs = 0;
% heights = 10:10:90;
heights = 20;
widths = 10:10:90;
scanStep = 10;

elapsedTim0 = etime(clock, startTime0)
%%==============================detect=====================================
% [bestRects,bestScores] = onePassDectionForInstDefaultParamsDisplay(103,dataDir,bgrDir)
startTime1 = clock;

%% initialize parameters
% PAD_SZ = 20;
PAD_SZ = 5;

% Thresholds to use when transforming masks to convert back to binary
MASK_ROT_THRESH = 0.75;
MASK_RSZ_THRESH = 0.75;

% Fraction of a rectangle which should be masked in as (padded) object for
% a rectangle to be considered
OBJ_MASK_THRESH = 0.5;

FEATSZ = 24;

% Make sure heights and widths are in ascending order, since this is a
% useful property we can exploit to speed some things up
heights = sort(heights);
widths = sort(widths);

%% Load grasping data. Loads into a 4-channel image where the first 3
% channels are RGB and the fourth is depth
I = graspPCDToRGBDImage(dataDir,instNum);
BG = double(imread(bgrFN));

elapsedTime1 = etime(clock, startTime1)

%% find object area's bounding box in the image
startTime2 = clock;
% Find the object in the image 
[M, bbCorners] = maskInObject(I(:,:,1:3),BG,PAD_SZ);

% Do a little processing on the depth data - find points that Kinect
% couldn't get, and eliminate additional outliers where Kinect gave
% obviously invalid values
D = I(:,:,4);
D = D*0;
DMask = D ~= 0;

[D,DMask] = removeOutliersDet(D,DMask,4);
[D,DMask] = removeOutliersDet(D,DMask,4);
D = smartInterpMaskedData(D,DMask);

%% display input image
fig11 = figure(11);
imshow(uint8(I(:,:,1:3)));
drawnow;

elapsedTime2 = etime(clock, startTime2)
%% convert RGB image to YUV

startTime3 = clock;

% Work in YUV color
I = rgb2yuv(I(:,:,1:3));

%% Pick out the area of the image corresponding to the object's (padded)
% bounding box to work with
objI = I(bbCorners(1,1):bbCorners(2,1),bbCorners(1,2):bbCorners(2,2),:);
objD = D(bbCorners(1,1):bbCorners(2,1),bbCorners(1,2):bbCorners(2,2),:);
objM = M(bbCorners(1,1):bbCorners(2,1),bbCorners(1,2):bbCorners(2,2),:);
objDM = DMask(bbCorners(1,1):bbCorners(2,1),bbCorners(1,2):bbCorners(2,2),:);

elapsedTime3 = etime(clock, startTime3)

%% initialize grasping rectangle
startTime4 = clock;

bestScore = -inf;
bestScore1 = [];
scoreMarks = [];

bestAng = 0;
bestW = 1;
bestH = 1;
bestR = 1;
bestC = 1;

% Precompute which widths we need to use with each height so we don't have
% to compute this in an inner loop
useWdForHt = false(length(heights),length(widths));

for i = 1:length(heights)
    useWdForHt(i,:) = widths > heights(i);
end

%% initialize the lines drawing grasping rectangle
prevLines = [];
bestLines = [];
barH = [];

IMask = ones(bbCorners(2,1)-bbCorners(1,1)+1,bbCorners(2,2)-bbCorners(1,2)+1);

%% browse image to detect grasping rectangle
for curAng = rotAngs
    % Rotate image to match the current angle. Threshold masks to keep them
    % binary
    curI = imrotate(objI,curAng);
    curD = imrotate(objD,curAng);
    curMask = imrotate(objM,curAng) > MASK_ROT_THRESH;
    curDMask = imrotate(objDM,curAng) > MASK_ROT_THRESH;
    curIMask = imrotate(IMask,curAng) > MASK_ROT_THRESH;
    
    % Compute surface normals. Only do this here to avoid having to rotate
    % the normals themselves when we rotate the image (can't just
    % precompute the normals and then rotate the "normal image" since the
    % axes change)
    curN = getSurfNorm(curD);
    
    curRows = size(curI,1);
    curCols = size(curI,2);
    
    %% ===========================Score Table==============================
    scoreTable = zeros(curRows, curCols)-10;
    fig2222 = figure(2222);
    %% ====================================================================
    
    % Going by the r/c dimensions first, then w/h should be more cache
    % efficient since it repeatedly reads from the same locations. Who
    % knows if that actually matters but the ordering's arbitrary anyway
    
    halfMinHeight = 5;
    halfMinWidth = 5;
    %% try rectangle center's coordinates
%     for rowCenter = PAD_SZ:scanStep:curRows-PAD_SZ
%         for colCenter = PAD_SZ:scanStep:curCols-PAD_SZ
    relativeRectCoord = [PAD_SZ PAD_SZ curRows-PAD_SZ curCols-PAD_SZ]; %firstRow, firstCol, lastRow, lastCol
    
    scoreMarks1 = [];
    for iteration = 1:8      
        if isempty(relativeRectCoord)
            continue;
        end
        segmentRow = round((relativeRectCoord(3) - relativeRectCoord(1))/3);
        segmentCol = round((relativeRectCoord(4) - relativeRectCoord(2))/3);
        
        subRects(1, :) = [relativeRectCoord(1) relativeRectCoord(2) relativeRectCoord(3)-segmentRow relativeRectCoord(4)-segmentCol];
        subRects(2, :) = [relativeRectCoord(1) relativeRectCoord(2)+segmentCol relativeRectCoord(3)-segmentRow relativeRectCoord(4)];
        subRects(3, :) = [relativeRectCoord(1)+segmentRow relativeRectCoord(2) relativeRectCoord(3) relativeRectCoord(4)-segmentCol];
        subRects(4, :) = [relativeRectCoord(1)+segmentRow relativeRectCoord(2)+segmentCol relativeRectCoord(3) relativeRectCoord(4)];
        
        tempScore = 0;
        tempSelectedRect = subRects(1, :);
        for rectCase = 1:4
            rowCenter = round((subRects(rectCase, 1) + subRects(rectCase, 3))/2);
            colCenter = round((subRects(rectCase, 2) + subRects(rectCase, 4))/2);
            
            for hh = 1:length(heights)
                heiRect = heights(hh);
                % ignore the invalid rectangle
                if (rowCenter-heiRect/2 < 1) | (rowCenter+heiRect/2 > curRows)
                    continue;
                end
                
                for widRect = widths(useWdForHt(hh,:))
                    % ignore the invalid 
                    if (widRect <= heiRect) | (colCenter-widRect/2 < 1) | (colCenter+widRect/2 > curCols)
                        continue;
                    end
                    
                    heiDiv2 = heiRect/2;
                    widDiv2 = widRect/2;
                    
                    tempRectInfo = [rowCenter-heiDiv2; colCenter-widDiv2; heiRect; widRect];
                    tempRectPoints = [rowCenter-heiDiv2 colCenter-widDiv2; rowCenter+heiDiv2 colCenter-widDiv2; rowCenter+heiDiv2 colCenter+widDiv2; rowCenter-heiDiv2 colCenter+widDiv2];
                    
                    % If the rectangle doesn't contain enough of the
                    % object (plus padding), move on because it's probably
                    % not a valid grasp regardless of score
                    if rectMaskFraction(curMask,tempRectInfo(1),tempRectInfo(2),tempRectInfo(3),tempRectInfo(4)) < OBJ_MASK_THRESH || cornerMaskedOut(curIMask,tempRectInfo(1),tempRectInfo(2),tempRectInfo(3),tempRectInfo(4))
                        continue;
                    end
                    
                    curScore = scoreRectangle(curI, curD, curN, curMask, curDMask, FEATSZ, MASK_RSZ_THRESH, featMeans, featStds, trainModes, w1, w2, w_class, tempRectInfo(1), tempRectInfo(2), tempRectInfo(3), tempRectInfo(4));
                    
                    scoreMarks = [scoreMarks; curScore];
                    scoreMarks1 = [scoreMarks1; curScore];
                    
                    if curScore > tempScore
                        tempScore = curScore;
                        tempSelectedRect = subRects(rectCase, :);
                    end
                    
                    rectPoints = tempRectPoints;
                    curRect = localRectToIm(rectPoints,curAng,bbCorners);
                    
                    %figure(1);
                    set(0, 'CurrentFigure', fig11);
                    removeLines(prevLines);
                    prevLines = plotGraspRect(curRect);
                    %delete(barH);
                    %barH = drawScoreBar(curScore,max(bestScore*1.1,1),20,320,30,300);
                    %figure(2);
                    %bar(w2Probs);
                    %axis([0 51 0 1]);
                    drawnow;

                    if curScore > bestScore
                        bestScore = curScore;
                        bestScore1 = [bestScore1 curScore];
                        bestAng = curAng;
                        bestR = rowCenter;
                        bestC = colCenter;
                        bestH = heiRect;
                        bestW = widRect;
                        
                        %figure(1);
                        set(0, 'CurrentFigure', fig11);
                        removeLines(bestLines);
                        bestLines = plotGraspRect(curRect,'g','y');
                        drawnow;
                        
                        if curScore > 6
                            rectPoints = round([bestR-bestH/2 bestC-bestW/2; bestR+bestH/2 bestC-bestW/2; bestR+bestH/2 bestC+bestW/2; bestR-bestH/2 bestC+bestW/2]);

                            bestRect = localRectToIm(rectPoints,bestAng,bbCorners);

                            figure(333);
                            plot(1:size(bestScore1, 2), bestScore1);
                            figure(444);
                            plot(1:size(scoreMarks, 1), scoreMarks);

                            elapsedTime4 = etime(clock, startTime4)
                            
                            return;
                        end
                    end                    
                    %% ================Update Score========================
%                     if curScore > scoreTable(rowCenter, colCenter) 
%                         scoreTable(rowCenter, colCenter) = curScore;
%                     end
                    %% ====================================================
                end
            end
        end
        
        relativeRectCoord = tempSelectedRect;
    end
%         end
%     end
    figure(555); plot(1:size(scoreMarks1, 1), scoreMarks1); grid on;
    set(0, 'CurrentFigure', fig2222);
    surf(scoreTable);
end

% removeLines(prevLines);

% Take the best rectangle params we found and convert to image space
% This is actually a little tricky because the image rotation operation
% isn't straighforward to invert
rectPoints = round([bestR-bestH/2 bestC-bestW/2; bestR+bestH/2 bestC-bestW/2; bestR+bestH/2 bestC+bestW/2; bestR-bestH/2 bestC+bestW/2]);

bestRect = localRectToIm(rectPoints,bestAng,bbCorners);

figure(333);
plot(1:size(bestScore1, 2), bestScore1);

elapsedTime4 = etime(clock, startTime4)
