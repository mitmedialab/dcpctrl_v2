% IMG2RASTTOOLPATH.M Converts image to raster toolpath

%{
img2rasttoolpath.m
Julian Leland, MIT Media Lab
2017-02-15

This function converts an image to a rasterized toolpath for the DCP.

%}

%% Setup
showfigs = 1; % Flag to display figures or suppress

plotres = 10; % Display every plotres'th point when doing large scatter plots

cmap = get(gca,'ColorOrder');
close all;

%% Import image and convert to grayscale, find edges
%in_img = imread('AltecPNG.png','png'); % Altec logo
in_img = imread('ADSKLogo_A.png','png'); % Autodesk 'A' logo
in_img_gs = rgb2gray(in_img);
bw_img = edge(in_img_gs,'canny');
if showfigs
    figure(1);
    subplot(3,1,1)
    imshow(in_img);
    subplot(3,1,2)
    imshow(in_img_gs);
    subplot(3,1,3)
    imshow(bw_img);
end

%% Define size of image, raster spacing, and generate waypoints for edges of raster
imrows = size(bw_img,1);
imcols = size(bw_img,2);

asr = imrows/imcols; % Aspect ratio of image
imh = 1000; % Height of image, mm.
imw = imh/asr;

rastspc = 100; % Spacing between raster lines, mm'
rastofst = 200; % Outwards offset to apply to image edges

xpos = 0; % Define x position of output image

% Extract row & column indices for white points in image
out_img = [];
[row,col] = find(bw_img == 1);
out_img = [zeros(length(row),1),col,row];

% For every row in the image, identify the left- and right-most white
% points
ledges = zeros(imrows,2);
redges = zeros(imrows,2);
for n = 1:imrows
    row_mask = [out_img(:,3) == n];
    if sum(row_mask) > 0
        ledges(n,:) = [min(out_img(row_mask,2)),n];
        redges(n,:) = [max(out_img(row_mask,2)),n];
    else
        ledges(n,:) = [NaN,NaN];
        redges(n,:) = [NaN,NaN];
    end
end

% Overlay plot to show edges & verify that they are correctly located.
if showfigs
    figure(2);
    imshow(in_img);
    hold on;
    scatter(ledges(:,1),ledges(:,2))
    scatter(redges(:,1),redges(:,2))
    hold off;
end

% Scale points to desired image size
ledges(:,1) = mapRange(ledges(:,1),0,imcols,0,imw);
ledges(:,2) = mapRange(ledges(:,2),imrows,0,0,imh);
redges(:,1) = mapRange(redges(:,1),0,imcols,0,imw);
redges(:,2) = mapRange(redges(:,2),imrows,0,0,imh);

% Add lateral offset to edges
ledges_ofst = bsxfun(@plus,ledges,[-rastofst, 0]);
redges_ofst = bsxfun(@plus,redges,[rastofst, 0]);

% Starting from bottom + some offset, select every rastspc point
ledges_cond = [];
redges_cond = [];
baseofst = 5; % Number of rows from bottom to offset

ymin = min(ledges_ofst(:,2)); % This is the smallest y-position *value* = the lowest point
ymax = max(ledges_ofst(:,2)); % This is the largest y-position *value* = the highest point

ymin_idx = find(ledges_ofst(:,2) == ymin); % This is the index of the smallest y-position value - generally large
ymax_idx = find(ledges_ofst(:,2) == ymax); % This is the index of the largest y-position value - generally small

ycur = ledges_ofst(ymin_idx-baseofst,2); % Start at the bottom of the image (smallest value, largest index minus some offset)

% Add the points at this level, plus the index where they show up
ledges_cond = [ledges_cond;[ledges_ofst(ymin_idx-baseofst,:),ymin_idx-baseofst]];
redges_cond = [redges_cond;[redges_ofst(ymin_idx-baseofst,:),ymin_idx-baseofst]];

% Now add additional points every rastspc mm above this point
for n = (ymin_idx-baseofst):-1:ymax_idx
    if ledges_ofst(n,2) >= ycur+rastspc
        ledges_cond = [ledges_cond;[ledges_ofst(n,:),n]];
        redges_cond = [redges_cond;[redges_ofst(n,:),n]];
        ycur = ledges_ofst(n,2);
    end
end

% Plot to show offset distance & final set of points
if showfigs
    figure(3);
    hold on;
    scatter(ledges(:,1),ledges(:,2),[],bsxfun(@times,ones(length(ledges),3),cmap(1,:)));
    scatter(ledges_ofst(:,1),ledges_ofst(:,2),[],bsxfun(@times,ones(length(ledges),3),cmap(2,:)));
    scatter(ledges_cond(:,1),ledges_cond(:,2),[],'k');
    scatter(redges(:,1),redges(:,2),[],bsxfun(@times,ones(length(ledges),3),cmap(1,:)));
    scatter(redges_ofst(:,1),redges_ofst(:,2),[],bsxfun(@times,ones(length(ledges),3),cmap(2,:)));
    scatter(redges_cond(:,1),redges_cond(:,2),[],'k');
    hold off;
end

%% Generate a set of waypoints that moves between all edge waypoints

traj_cond = {}; % Empty cell array to store trajectories

% Create a complete, ordered list of waypoints, starting at bottom left and
% moving to top right.
waypts_list = []; % Empty vector for waypoints
for n = 1:2:length(ledges_cond);
    if length(ledges_cond) < n+1 % If we're at an odd-numbered segment
        waypts_list = [waypts_list;ledges_cond(n,:);redges_cond(n,:)];
    else
        waypts_list = [waypts_list;ledges_cond(n,:);redges_cond(n,:);redges_cond(n+1,:);ledges_cond(n+1,:)];
    end
end

cartwaypts = [waypts_list(:,1),waypts_list(:,2),zeros(size(waypts_list,1),1)]; % Add Z-position to waypoints list - assume zero.

if showfigs
    figure(4);
    hold on;
    scatter(ledges_cond(:,1),ledges_cond(:,2),[],'k');
    scatter(redges_cond(:,1),redges_cond(:,2),[],'k');
    plot(cartwaypts(:,1),cartwaypts(:,2),'r')
    hold off;
end

% Create waypoints vector for trajectory generation with empty zeroes for
% tool commands. Since we assume light painting, leave spaces for H,S,V,K
trajwaypts = {cartwaypts,[0,0,0,0]};

%% Create trajectory
% Note: Increasing tacc creates gentler curves at corners
traj_cond = waypts2carttraj(trajwaypts,0.01,100,1);

% Plot over image
if showfigs
    figure(5);
    hold on;
    scatter(ledges_cond(:,1),ledges_cond(:,2),[],'k');
    scatter(redges_cond(:,1),redges_cond(:,2),[],'k');
    scatter(traj_cond{1}(:,1),traj_cond{1}(:,2),0.5,'b')
    hold off;
end

%% Identify correct tool color at each trajectory waypoint
% Determine how much we need to scale input image by
in_img_hsv = rgb2hsv(in_img);
scale_img = imresize(in_img_hsv,imh/imrows,'nearest'); % Need to do nearest resizing, otherwise we get HSV values outside of permissible range

if showfigs
    figure(6);
    imshow(scale_img,'YData',[imh 1]);
    hold on;
    scatter(traj_cond{1}(1:plotres:end,1),traj_cond{1}(1:plotres:end,2),1,'w')
    hold off;
end

% Note: It turns out that increasing the raster resolution doesn't
% substantially increase the time required to calculate traj_cond. We'll
% just do every point = full resolution.
%rasterres = 1; % Resolution of raster. Light value will be sampled at every nth point

for n = 1:length(traj_cond{1})
    xycoord = round(traj_cond{1}(n,1:2));
    if xycoord(1) > size(scale_img,2) || xycoord(1) < 1 % If we're outside the bounds of the image
       traj_cond{2}(n,:) = [0,0,0,2500]; 
    else
        traj_cond{2}(n,:) = [scale_img(size(scale_img,1)-xycoord(2),xycoord(1),1),scale_img(size(scale_img,1)-xycoord(2),xycoord(1),2),scale_img(size(scale_img,1)-xycoord(2),xycoord(1),3),2500];
    end
end

% Plotting large scatter plots is obnoxious - use plotres toreduce density.

if showfigs
    figure(7);
    hold on;
    scatter(traj_cond{1}(1:plotres:end,1),traj_cond{1}(1:plotres:end,2),[],[hsv2rgb(traj_cond{2}(1:plotres:end,1:3))]);
end

%% Re-order trajectory so that it matches AT40GW plane (YZ plane)
traj_reord = {};
traj_reord{1} = [traj_cond{1}(:,3),traj_cond{1}(:,1),traj_cond{1}(:,2),traj_cond{1}(:,6),traj_cond{1}(:,4),traj_cond{1}(:,5),traj_cond{1}(:,7)];
traj_reord{2} = traj_cond{2};

%% Convert task-space trajectory to joint space
qtraj = {carttraj2jointtraj_at40gw(robot,traj_reord{1})};

qtrajs = {qtraj{1}(:,1:4)};
qdtrajs = {qtraj{1}(:,5:8)};
qrawtrajs = {j2rfcn_at40gw(qtraj{1}(:,1),qtraj{1}(:,2),qtraj{1}(:,3),qtraj{1}(:,4))};
qdrawtrajs = {jointvel2rawvel_at40gw(robot,qtraj{1}(:,1:4),qtraj{1}(:,5:8))};
tooltrajs = {traj_reord{2}};


% Edit: not in future, not doing this here. Going to implement this in a different
% function, so we can separate Cartesian trajectory generation from
% joint-space trajectory conversion.

%% Clean up workspace
%{
xtraj = traj_reord;
input('Trajectory generation complete.\nPlease press ENTER to close figures and clear unused variables, or Ctrl-C to break:');
clearvars -except xtraj robot
close all;
%}
