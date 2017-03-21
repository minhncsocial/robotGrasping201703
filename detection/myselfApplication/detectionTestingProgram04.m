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

% addpath ../detectionUtils/
% addpath ../../util

dataDir = 'E:\WORK\ORGANIZATION\NTUT\Robot Grasping\Project02\Code\rawDataSet';
bgrDir = 'E:\WORK\ORGANIZATION\NTUT\Robot Grasping\Project02\Code\rawDataSet';
instNum = sscanf('pcd0105r.png', '%*3c%u')

%% load need data for detection
load ../../data/bgNums.mat
load ../../data/graspModes24.mat
load ../../weights/graspWFinal.mat
load ../../data/graspWhtParams.mat

%% config parameters for detection
% Just use the positive-class weights for grasping.
w_class = w_class(:,1);

bgrFN = sprintf('%s/pcdb%04dr.png',bgrDir,bgNo(instNum));

rotAngs = 0;
heights = 10:10:90;
widths = 10:10:90;
scanStep = 10;

elapsedTim0 = etime(clock, startTime0)
%%==============================detect=====================================
% [bestRects,bestScores] = onePassDectionForInstDefaultParamsDisplay(103,dataDir,bgrDir)
startTime1 = clock;

%% initialize parameters
% PAD_SZ = 20;
PAD_SZ = 10;

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
DMask = D ~= 0;

[D,DMask] = removeOutliersDet(D,DMask,4);
[D,DMask] = removeOutliersDet(D,DMask,4);
D = smartInterpMaskedData(D,DMask);

%% display input image
fig1 = figure(1);
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

bestAng = -1;
bestW = -1;
bestH = -1;
bestR = -1;
bestC = -1;

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
    
    % Going by the r/c dimensions first, then w/h should be more cache
    % efficient since it repeatedly reads from the same locations. Who
    % knows if that actually matters but the ordering's arbitrary anyway
    
    halfMinHeight = 5;
    halfMinWidth = 5;
    %% try rectangle center's coordinates
    for rowCenter = halfMinHeight:scanStep:curRows-halfMinHeight
        for colCenter = halfMinWidth:scanStep:curCols-halfMinWidth
            for heiRect = 10:10:90
                % ignore the invalid rectangle
                if (rowCenter-heiRect/2 < 1) | (rowCenter+heiRect/2 > curRows)
                    continue;
                end
                
                for widRect = 10:10:90
                    % ignore the invalid 
                    if (colCenter-widRect/2 < 1) | (colCenter+widRect/2 > curCols)
                        continue;
                    end
                    
                    heiDiv2 = heiRect/2;
                    widDiv2 = widRect/2;
                    
                    % Have a valid candidate rectangle
                    % Extract features for the current rectangle into the
                    % format the DBN expects
                    [curFeat, curFeatMask] = featForRect(curI,curD,curN,curDMask,rowCenter-heiDiv2,colCenter-widDiv2,heiRect,widRect,FEATSZ,MASK_RSZ_THRESH);
                    curFeat = simpleWhiten(curFeat,featMeans,featStds);
                    curFeat = scaleFeatForMask(curFeat, curFeatMask, trainModes);
                    
                    % Run the features through the DBN and get a score.
                    % Might be more efficient to collect features for a
                    % group of rectangles and run them all at once
                    w1Probs = 1./(1+exp(-[curFeat 1]*w1));
                    w2Probs = 1./(1+exp(-[w1Probs 1]*w2));
                    curScore = [w2Probs 1]* w_class;
                    
                    rectPoints = [rowCenter-heiDiv2 colCenter-widDiv2; rowCenter+heiDiv2 colCenter-widDiv2; rowCenter+heiDiv2 colCenter+widDiv2; rowCenter-heiDiv2 colCenter+widDiv2];
                    curRect = localRectToIm(rectPoints,curAng,bbCorners);
                    
                    %figure(1);
                    set(0, 'CurrentFigure', fig1);
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
                        bestAng = curAng;
                        bestR = rowCenter;
                        bestC = colCenter;
                        bestH = heiRect;
                        bestW = widRect;
                        
                        %figure(1);
                        removeLines(bestLines);
                        bestLines = plotGraspRect(curRect,'g','y');
                        drawnow;
                    end
                end
            end
        end
    end
end

% removeLines(prevLines);

% Take the best rectangle params we found and convert to image space
% This is actually a little tricky because the image rotation operation
% isn't straighforward to invert
rectPoints = [rowCenter-heiDiv2 colCenter-widDiv2; rowCenter+heiDiv2 colCenter-widDiv2; rowCenter+heiDiv2 colCenter+widDiv2; rowCenter-heiDiv2 colCenter+widDiv2];

bestRect = localRectToIm(rectPoints,bestAng,bbCorners);

elapsedTime4 = etime(clock, startTime4)
