%% first init
startTime0 = clock; % Start timer

path_detectionUtils = 'E:\WORK\ORGANIZATION\NTUT\Robot Grasping\Project02\Code\deepGraspingCode1\detection\detectionUtils';
path_utils = 'E:\WORK\ORGANIZATION\NTUT\Robot Grasping\Project02\Code\deepGraspingCode1\util';
addpath(path_detectionUtils);
addpath(path_utils);
addpath('myselfUtil');

% load need data for detection
load ../../data/bgNums.mat
load ../../data/graspModes24.mat
load ../../weights/graspWFinal.mat
load ../../data/graspWhtParams.mat

% config parameters for detection
% Just use the positive-class weights for grasping.
w_class = w_class(:,1);

rotAngs = 0:15:(11*15);
heights = 30;
widths = 10:10:90;
scanStep = 10;

% initialize parameters
% PAD_SZ = 20;
PAD_SZ = 20;

% Thresholds to use when transforming masks to convert back to binary
MASK_ROT_THRESH = 0.75;
MASK_RSZ_THRESH = 0.75;

% Fraction of a rectangle which should be masked in as (padded) object for
% a rectangle to be considered
OBJ_MASK_THRESH = 0.5;

FEATSZ = 24;

elapsedTim0 = etime(clock, startTime0) % Display time

%% Load Image and Background
startTime1 = clock; % Start timer

dataDir = 'E:\WORK\ORGANIZATION\NTUT\Robot Grasping\Project02\Code\rawDataSet';
bgrDir = 'E:\WORK\ORGANIZATION\NTUT\Robot Grasping\Project02\Code\rawDataSet';

% Select image
instNum = sscanf('pcd0111r.png', '%*3c%u');

if instNum < 2000
    bgrFN = sprintf('%s/pcdb%04dr.png',bgrDir,bgNo(instNum));
else
    bgrFN = sprintf('%s/pcdb%04dr.png',bgrDir, 2002);
end

% Load grasping data. Loads into a 4-channel image where the first 3
% channels are RGB and the fourth is depth
I = graspPCDToRGBDImage(dataDir,instNum);
BG = double(imread(bgrFN));

elapsedTime1 = etime(clock, startTime1) % Display time

%% Find object area's bounding box in the image
startTime2 = clock;
% Find the object in the image % Start timer 
[M, bbCorners] = maskInObject(I(:,:,1:3),BG,PAD_SZ);

% display input image
fig11 = figure(11);
imshow(uint8(I(:,:,1:3)));
drawnow;

elapsedTime2 = etime(clock, startTime2) % Display time

%% convert RGB image to YUV
startTime3 = clock;

% Work in YUV color
I = rgb2yuv(I(:,:,1:3));

% Pick out the area of the image corresponding to the object's (padded)
% bounding box to work with
objI = I(bbCorners(1,1):bbCorners(2,1),bbCorners(1,2):bbCorners(2,2),:);
objM = M(bbCorners(1,1):bbCorners(2,1),bbCorners(1,2):bbCorners(2,2),:);

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

% initialize the lines drawing grasping rectangle
prevLines = [];
bestLines = [];
barH = [];

IMask = ones(bbCorners(2,1)-bbCorners(1,1)+1,bbCorners(2,2)-bbCorners(1,2)+1);
goldenRatio = (sqrt(5) - 1) / 2;