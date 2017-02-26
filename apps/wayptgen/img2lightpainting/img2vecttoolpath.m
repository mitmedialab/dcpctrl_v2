% IMG2VECTTOOLPATH Converts an image to a vector toolpath for the DCP

%{
Julian Leland, MIT Media Lab, 2016-08-02
Imported to dcpctrl_v2 2017-01-23

Edited to comply with Trajectory Input Standard Definition (02-2017) on
2017-02-24

%}

%% Get list of all variables currently in workspace
curvars = who; % Get current variables

%% Set up user input parameters
vizflag = 1; % Flag to show figures or suppress

imh = 1500; % Height of image, mm.

q = 20; % Pick every qth waypoint to actually draw when reducing waypoint depth

subsegt_lenthresh = 5; % If a subsegment is shorter than this number of segments, reject it.

subsegt_dist = 100; % Permissible distance in mm between subsegments for which no intermediate path will be made.

% Define move parameters
dt = 0.01; % Desired timestep, seconds.
tacc = 0.01; % Desired Cartesian acceleration time (mm/s^2)
spd = 100; % Desired Cartesian velocity, (mm/s)

%% Import image and convert to grayscale
%in_img = imread('AltecPNG.png','png'); % Altec logo
in_img = imread('ADSKLogo_A.png','png'); % Autodesk 'A' logo 
%in_img = imread('ReebokDelta.png','png'); % Reebok Delta logo
in_img = rgb2gray(in_img);
bw_img = edge(in_img,'canny');
if vizflag
    figure(1);
    imshow(bw_img);
end

%% Define desired output size of image
asr = size(bw_img,1)/size(bw_img,2); % Aspect ratio of image
imw = imh/asr;

xpos = 0; % Define x position of output image

%% Extract row & column indices for white points in image
out_img = [];
[row,col] = find(bw_img == 1);
out_img = [zeros(length(row),1),col,row];

%% Map image onto desired size range
out_img(:,2) = mapRange(out_img(:,2),min(out_img(:,2)),max(out_img(:,2)),0,imw);
out_img(:,3) = mapRange(out_img(:,3),max(out_img(:,3)),min(out_img(:,3)),0,imh);

%% OPTIONAL: Plot each point in sequence to see what the system is going to print
if vizflag
    figure(2)
    scatter3(out_img(:,1),out_img(:,2),out_img(:,3),[],hsv(length(out_img)));
    hold on;
    plot3(out_img(1,1),out_img(1,2),out_img(1,3),'mx');
    hold off;
    %{
    figure(3);
    hold on;
    for n = 1:length(out_img)
        scatter3(out_img(n,1),out_img(n,2),out_img(n,3),'b');
        drawnow;
    end
    hold off;
    %}
end

%% Set start point and re-order points to draw in sequence
% Method from http://stackoverflow.com/questions/11631934/sort-coordinates-points-in-matlab
dist = pdist2(out_img,out_img);

N = size(out_img,1);
result = NaN(1,N);
result(1) = 1;

for n = 2:N
    dist(:,result(n-1)) = Inf;
    [~, closest_idx] = min(dist(result(n-1),:));
    result(n) = closest_idx;
end

result = result';

%% OPTIONAL: Plot each point in sequence to make sure that system is filtering correctly
% Commented out because it's super slow
if vizflag
    %{
    figure(4);
    hold on;
    for n = 1:length(out_img)
        scatter3(out_img(result(n),1),out_img(result(n),2),out_img(result(n),3),'b');
        drawnow;
    end
    hold off;
    %}
end

%% Sort out_img by result
out_img_sorted = [];
for n = 1:length(out_img)
    out_img_sorted = [out_img_sorted;out_img(result(n),:)];
end

%% OPTIONAL: Plot each point in sequence to make sure that system is filtering correctly
if vizflag
    %{
    figure(5);
    view([60,20]);
    hold on;
    for n = 1:length(out_img_sorted)
        scatter3(out_img_sorted(n,1),out_img_sorted(n,2),out_img_sorted(n,3),'b');
        drawnow;
    end
    hold off;
    %}
end

%% Identify mean distance between points, and use this to set flags indicating when new section of image has been reached
segflag = zeros(length(out_img_sorted),1);
%meandist = mean(diff(out_img_sorted));
ptdist = [0];
for n = 1:length(out_img_sorted)-1
    ptdist = [ptdist,pdist2(out_img_sorted(n,:),out_img_sorted(n+1,:))];
end
ptdist = ptdist';
meandist = mean(ptdist);
dist_filt = 5; % Distances between points must be greater than dist_filt*meandist to trigger a new segment
for n = 1:length(out_img_sorted)
    if ptdist(n) >= meandist*dist_filt % If the current distance between points is greater than the mean distance between points
        segflag(n) = 1;
    end
end

out_img_sorted = [out_img_sorted,ptdist,segflag];

%% Pick every qth point to actually draw (reduce density of points)
out_img_sm = [];
for n = 1:length(out_img_sorted)
    if mod(n,q) == 0
        out_img_sm = [out_img_sm;out_img_sorted(n,:)];
    end
    if out_img_sorted(n,5) == 1
        out_img_sm = [out_img_sm;out_img_sorted(n,:)];
    end
end

%% OPTIONAL: Plot each point in sequence to make sure that system is filtering correctly
if vizflag
    figure(6);
    view([60,20]);
    hold on;
    for n = 1:length(out_img_sm)
        scatter3(out_img_sm(n,1),out_img_sm(n,2),out_img_sm(n,3),'b');
        drawnow;
    end
    axis equal
    axis vis3d
    hold off;
end

%% Split up into subsegments with index
subsegts = {[]};
segt_ct = 1;
for n = 1:length(out_img_sm)
    if out_img_sm(n,5) == 1
        % disp('New segment!');
        segt_ct = segt_ct + 1;
        subsegts{segt_ct,1} = [];
    end
    subsegts{segt_ct,1} = [subsegts{segt_ct};out_img_sm(n,:)];
end

%% Reject subsegments that are too short
subsegts_long = {};
for n = 1:size(subsegts,1)
    if size(subsegts{n},1) >= subsegt_lenthresh
        subsegts_long = [subsegts_long;subsegts{n}];
    end
end

%% For each subsegment, add the first point back in to the end to close each segment
% NOTE: This only really works for subsegments that are closed loops.
% Linear segments won't play nice with this.
for n = 1:length(subsegts_long)
    subsegts_long{n} = [subsegts_long{n};subsegts_long{n}(1,:)];
    subsegts_long{n}(1,5) = 1; % Set flag in first row to 1
    subsegts_long{n}(end,5) = 1; % Set flag in last row to 1
end

%% Create segments to path between subsegments
subsegts_join = {};
for n = 1:size(subsegts_long,1)-1
    subsegts_join = [subsegts_join;subsegts_long{n}];
    % If the distance in any direction between the end of the current
    % subsegment and the start of the next is greater than our threshold,
    % then add an extra segment
    if any(abs(subsegts_long{n}(end,:)-subsegts_long{n+1}(1,:)) >= subsegt_dist)
        join_segt = [subsegts_long{n}(end,:);subsegts_long{n+1}(1,:)];
        subsegts_join = [subsegts_join; join_segt];
    end
end

% Add in last subsegment
subsegts_join = [subsegts_join;subsegts_long{end}];
%% Turn into waypts
waypts = {}; % Keep waypoints for each segment separated
waypts_comb = []; % Collect all waypoints into single trajectory

for n = 1:length(subsegts_join)
    waypts{n,1} = subsegts_join{n}(:,1:3);
    waypts_comb = [waypts_comb;subsegts_join{n}(:,1:3)];
end

% At this point, waypts has the format (x,y,z). These are not
% time-parametrized, and they don't include a tool vector yet (we will add
% manually).

%% Convert waypts to DCP task trajectory
% Add dummy vectors to waypts
for n = 1:size(waypts,1)
    % This creates dummy trajectories for the AT40, KUKA , tool and enable
    waypts(n,2:5) = {[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0]};
end

%{
xtraj = {}; % Generate empty cell array to hold DCP xtraj
for n = 1:size(waypts,1)
     xtraj(n,:) = waypts2carttraj(waypts(n,:),dt,spd,tacc);
end
%}

% New struct format
xtraj_temp = {}; % Generate empty cell array to hold DCP xtraj
for n = 1:size(waypts,1)
     xtraj_temp(n,:) = waypts2carttraj(waypts(n,:),dt,spd,tacc);
end

xtraj = struct('t',xtraj_temp(:,1),'dcp',xtraj_temp(:,2),'at40',xtraj_temp(:,3),'kuka',xtraj_temp(:,4),'tool',xtraj_temp(:,5),'en',xtraj_temp(:,6));


%% Create real AT40GW task trajectory
% Just copy the DCP move to the AT40GW, since the AT40 is the only part
% that is moving

%xtraj(:,3) = xtraj(:,2);
for n = 1:size(xtraj,1)
    xtraj(n).at40 = xtraj(n).dcp;
end

%% Set tool trajectory
% For this particular trajectory, we assume that every odd row is ON, while
% every even row is OFF
for n = 1:size(xtraj,1)
    if mod(n,2) % We are on an odd trajectory
        % xtraj{n,5}(:,3) = 1;
        xtraj(n).tool(:,3) = 1;
    end
end

%% Set enable trajectory
for n = 1:size(xtraj,1)
    xtraj(n).en(:,1) = 1; % AT40GW is active
    xtraj(n).en(:,3) = 1; % Tool is active 
end

%% OPTIONAL: Check trajectory to make sure it makes sense
if vizflag
    figure(7);
    view([60,20]);
    axis square
    axis vis3d
    hold on
    for n = 1:size(xtraj,1)
        if xtraj(n).tool(1,3) == 1
            color = 'r';
        else
            color = 'b';
        end
        scatter3(xtraj(n).dcp(:,1), xtraj(n).dcp(:,2), xtraj(n).dcp(:,3), color);
        drawnow;
        pause(0.2);
    end
    hold off
end

%% Clear all other variables except for xtraj; close figures
curvars = {curvars{:},'xtraj'};
clearvars('-except', curvars{:});
clear curvars
close all;