% IMG2RASTTOOLPATH.M Converts image to raster toolpath

%{
img2rasttoolpath.m
Julian Leland, MIT Media Lab
2017-02-15

This function converts an image to a rasterized toolpath for the DCP.

Edited to comply with Trajectory Input Standard Definition (02-2017) on
2017-02-25

%}

%% Get list of all variables currently in workspace
curvars = who; % Get current variables

%% Setup
vizflag = 1; % Flag to display figures or suppress

imh = 2000; % Height of image, mm.

rastspc = 100; % Spacing between raster lines, mm'
rastofst = 200; % Outwards offset to apply to image edges

plotres = 10; % Display every plotres'th point when doing large scatter plots

% Motion parameters
dt = 0.1; % Timestep for trajectory, s.
spd = 120; % Cartesian velocity for trajectory, distance units/s
tacc = 1; % Cartesian acceleration time, distance units/s^2.

cmap = get(gca,'ColorOrder');
close all;

%% Import image and convert to grayscale, find edges
%in_img = imread('AltecPNG.png','png'); % Altec logo
in_img = imread('ADSKLogo_A.png','png'); % Autodesk 'A' logo
%in_img = imread('ReebokDelta.png','png'); % Reebok Delta logo
%in_img = imread('MIT_logo.png','png'); % MIT logo
in_img_gs = rgb2gray(in_img);
bw_img = edge(in_img_gs,'canny');
if vizflag
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
imw = imh/asr;

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
if vizflag
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
if vizflag
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

cartwaypts = [zeros(size(waypts_list,1),1),waypts_list(:,1),waypts_list(:,2)]; % Add X-position to waypoints list - assume zero.

if vizflag
    figure(4);
    hold on;
    scatter(ledges_cond(:,1),ledges_cond(:,2),[],'k');
    scatter(redges_cond(:,1),redges_cond(:,2),[],'k');
    plot(cartwaypts(:,2),cartwaypts(:,3),'r')
    hold off;
end

%% Convert waypts to DCP task trajectory
% This creates dummy trajectories for the AT40, KUKA , tool and enable
waypts = {cartwaypts,[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0]};

% Create trajectory
% Note: Increasing tacc creates gentler curves at corners
traj_cond = waypts2carttraj(waypts,dt,spd,tacc);

% Plot over image
if vizflag
    figure(5);
    hold on;
    scatter(ledges_cond(:,1),ledges_cond(:,2),[],'k');
    scatter(redges_cond(:,1),redges_cond(:,2),[],'k');
    scatter(traj_cond{2}(:,2),traj_cond{2}(:,3),0.5,'b')
    hold off;
end


%% Create real AT40GW task trajectory
% Just copy the DCP move to the AT40GW, since the AT40 is the only part
% that is moving

traj_cond{3} = traj_cond{2};


%% Identify correct tool color at each trajectory waypoint
% Determine how much we need to scale input image by
in_img_hsv = rgb2hsv(in_img);
scale_img = imresize(in_img_hsv,imh/imrows,'nearest'); % Need to do nearest resizing, otherwise we get HSV values outside of permissible range

if vizflag
    figure(6);
    imshow(scale_img,'YData',[imh 1]);
    hold on;
    scatter(traj_cond{2}(1:plotres:end,2),traj_cond{2}(1:plotres:end,3),1,'w')
    hold off;
end

% Note: It turns out that increasing the raster resolution doesn't
% substantially increase the time required to calculate traj_cond. We'll
% just do every point = full resolution.
%rasterres = 1; % Resolution of raster. Light value will be sampled at every nth point

for n = 1:length(traj_cond{2})
    xycoord = round(traj_cond{2}(n,2:3));
    if xycoord(1) > size(scale_img,2) || xycoord(1) < 1 % If we're outside the bounds of the image
       traj_cond{5}(n,:) = [0,0,0,2500,0,0]; 
    else
        traj_cond{5}(n,:) = [scale_img(size(scale_img,1)-xycoord(2),xycoord(1),1),scale_img(size(scale_img,1)-xycoord(2),xycoord(1),2),scale_img(size(scale_img,1)-xycoord(2),xycoord(1),3),2500,0,0];
    end
end

% Plotting large scatter plots is obnoxious - use plotres toreduce density.

if vizflag
    figure(7);
    hold on;
    scatter(traj_cond{2}(1:plotres:end,2),traj_cond{2}(1:plotres:end,3),[],[hsv2rgb(traj_cond{5}(1:plotres:end,1:3))]);
end

%% Convert trajectory to structure
xtraj = struct('t',traj_cond(:,1),'dcp',traj_cond(:,2),'at40',traj_cond(:,3),'kuka',traj_cond(:,4),'tool',traj_cond(:,5),'en',traj_cond(:,6));

%% Set enable trajectory
for n = 1:size(xtraj,1)
    xtraj(n).en(:,1) = 1; % AT40GW is active
    xtraj(n).en(:,3) = 1; % Tool is active 
end

%% OPTIONAL: Check trajectory to make sure it makes sense
if vizflag
    finalXtraj = figure;
    view([60,20]);
    axis square
    axis vis3d
    hold on
    for n = 1:size(xtraj,1)
        scatter3(xtraj(n).dcp(1:plotres:end,1), xtraj(n).dcp(1:plotres:end,2), xtraj(n).dcp(1:plotres:end,3), 10.*(xtraj(n).tool(1:plotres:end,3)+0.1), hsv2rgb(xtraj(n).tool(1:plotres:end,1:3)));
        drawnow;
        pause(0.2);
    end
    hold off
end

%% Clear all other variables except for xtraj; close figures
curvars = {curvars{:},'xtraj'};
clearvars('-except', curvars{:});
clear curvars
%close all;
close([1 2 3 4 5 6 7]); % Leave figure 8 open so we can see rough idea of trajectory colors
