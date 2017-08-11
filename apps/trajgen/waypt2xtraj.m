% waypt2xtraj Generalized toolpath importer for DCP v.2, for use with
% xtraj2qtraj, qtraj2slxtraj, etc.

%{
waypt2xtraj.m 
Julian Leland, MIT Media Lab 
2017-07-31

waypt2xtraj takes in generalized DCP waypoint cells and converts them to
xtraj format, suitable for use with the xtraj2qtraj/qtraj2slxtraj
toolchain.

%}

%% Get list of all variables currently in workspace
curvars = who; % Get current variables

%% Setup

% Motion parameters. These will not be used forever - eventually will
% switch to using input timesteps
dt = 0.1; % Timestep for trajectory, s.
spd = 150; % Cartesian velocity for trajectory, distance units/s
tacc = 10; % Cartesian acceleration time, distance units/s^2.

% Toolpath segmentation parameters
% WARNING: Path segmentation super experimental/hacky. DO NOT USE
pathSegmentation = 0; % Turn toolpath segmentation based on size on (1) or off (0).
sizeCutoff = 800; % Cutoff size in mm for segmenting trajectories between KUKA and AT40GW.
% Assumes that KUKA error compensation is on. Defined as diagonal length of
% box containing all points in a segment. If KUKA position correction
% limits are ±250 mm, then this should be smaller than norm([500,500,500])

plotres = 10;

vizflag = 1;

%% Extract position and time from waypts
waypts_pos = {};
waypts_t = {};
xtraj_temp = {}; % Generate empty cell array to hold DCP xtraj

% Extract time and position from waypts
for n = 1:size(waypts,1)
    waypts_pos{n,1} = waypts{n}(:,2:4);
    waypts_t{n,1} = waypts{n}(:,1);
end


%% Feed into waypts2carttraj to calculate AT40GW trajectory
% This section uses the defined speed and acceleration values to create a
% trajectory for the AT40GW (even though it's returned for the DCP - we
% move it in the next section). We will then calculate an "ideal" DCP
% trajectory as well, which we can use for real-time error compensation.

% Add dummy vectors to waypts
for n = 1:size(waypts_pos,1)
    % This creates dummy trajectories for the AT40, KUKA , tool and enable
    waypts_pos(n,2:5) = {[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0]};
end

% New struct format
xtraj_temp = {}; % Generate empty cell array to hold DCP xtraj
% for n = 1:size(waypts_pos,1)
%      xtraj_temp(n,:) = waypts2carttraj(waypts_pos(n,:),dt,spd,tacc);
% end

if pathSegmentation
    for n = 1:size(waypts_pos,1)
        % Measure the diagonal of the box that encloses the entire trajectory
        segDiagonal = norm([max(waypts_pos{n,1}(:,1))-min(waypts_pos{n,1}(:,1)),max(waypts_pos{n,1}(:,2))-min(waypts_pos{n,1}(:,2)),max(waypts_pos{n,1}(:,3))-min(waypts_pos{n,1}(:,3))]);
        if segDiagonal <= sizeCutoff
            % The segment is smaller than the sizeCutoff. Path to the KUKA.
            xtraj_temp(n,:) = waypts2carttraj(waypts_pos(n,:),dt,spd,1);
        else
            % The segment is larger than the sizeCutoff. Path to the AT40GW.
            xtraj_temp(n,:) = waypts2carttraj(waypts_pos(n,:),dt,spd,tacc);;
        end
    end
else
    for n = 1:size(waypts_pos,1)
        xtraj_temp(n,:) = waypts2carttraj(waypts_pos(n,:),dt,spd,tacc);
    end
end

%% Create real AT40GW task trajectory
% Just copy the DCP move to the AT40GW, since the AT40 is the only part
% that is moving

xtraj_temp(:,3) = xtraj_temp(:,2);

%% Develop "ideal" DCP trajectory
% We want the DCP trajectory to represent what we originally commanded the
% DCP to do, but generated at each instant in time.

for n = 1:size(xtraj_temp,1)
    
    % Calculate the distances between each point in the waypoints
    waypt_dists = [];
    for m = 1:size(waypts_pos{n},1)-1 % For each waypoint in this segment
        tempdist = norm(waypts_pos{n}(m+1,:) - waypts_pos{n}(m,:));
        waypt_dists = [waypt_dists;tempdist];
    end
    
    % Sum these distances to return total distance at each waypoint
    totalwayptdist = [0];
    for m = 2:size(waypt_dists,1)+1
        totalwayptdist(m,1) = waypt_dists(m-1,1)+totalwayptdist(m-1,1);
    end
    
    % Now repeat for xtraj segments
    % Calculate the distances between each point in the xtraj segment
    traj_dists = [];
    for m = 1:size(xtraj_temp{n,2},1)-1 % For each waypoint in this segment
        tempdist = norm(xtraj_temp{n,2}(m+1,1:3) - xtraj_temp{n,2}(m,1:3));
        traj_dists = [traj_dists;tempdist];
    end
    
    % Sum these distances to return total distance at each point in the
    % segment
    totaltrajdist = [0];
    for m = 2:size(traj_dists,1)+1
        totaltrajdist(m,1) = traj_dists(m-1,1)+totaltrajdist(m-1,1);
    end
    
    % Calculate percentage distances for total trajectory distances and
    % total waypt distances
    pctwayptdist = totalwayptdist/max(totalwayptdist);
    pcttrajdist = totaltrajdist/max(totaltrajdist);
    
    % Interpolate waypoint positions and waypoint percentage distances
    % against trajectory percentage distances
    traj_posDes = interp1(pctwayptdist,waypts_pos{n},pcttrajdist);
    
    % Naive calculation of Cartesian velocity using diff
    traj_velDes = [diff(traj_posDes);0,0,0];

    xtraj_temp{n,2} = [traj_posDes,traj_velDes];
end

%% Re-insert correct tool trajectory
% For now, we're just inserting the first set of tool commands for each
% segment for the entire segment. Eventually, we will map tool commands
% from the waypoints to xtraj_temp by time

for n = 1:size(xtraj_temp,1)
    xtraj_temp{n,5} = repmat(waypts{n}(1,5:10),size(xtraj_temp{n,5},1),1);
end

%% Set enable trajectories appropriately
% Here, we use sizeCutoff to either turn the KUKA on and AT40GW off, or
% vis-versa

if pathSegmentation
    for n = 1:size(xtraj_temp,1)
        % Measure the diagonal of the box that encloses the entire trajectory
        segDiagonal = norm([max(xtraj_temp{n,2}(:,1))-min(xtraj_temp{n,2}(:,1)),max(xtraj_temp{n,2}(:,2))-min(xtraj_temp{n,2}(:,2)),max(xtraj_temp{n,2}(:,3))-min(xtraj_temp{n,2}(:,3))]);
        if segDiagonal <= sizeCutoff
            % The segment is smaller than the sizeCutoff. Path to the KUKA.
            xtraj_temp{n,6} = repmat([0 1 1],size(xtraj_temp{n,6},1),1);
            xtraj_temp{n,3} = repmat(xtraj_temp{n,3}(1,:),size(xtraj_temp{n,3},1),1); % Artificially command AT40GW not to move
        else
            % The segment is larger than the sizeCutoff. Path to the AT40GW.
            xtraj_temp{n,6} = repmat([1 0 1],size(xtraj_temp{n,6},1),1);
        end
    end
else
    for n = 1:size(xtraj_temp,1)
        xtraj_temp{n,6} = repmat([1 0 1],size(xtraj_temp{n,6},1),1);
    end
end
    
%% Convert trajectory to structure

xtraj = struct('t',xtraj_temp(:,1),'dcp',xtraj_temp(:,2),'at40',xtraj_temp(:,3),'kuka',xtraj_temp(:,4),'tool',xtraj_temp(:,5),'en',xtraj_temp(:,6));

%% OPTIONAL: Check trajectory to make sure it makes sense
if vizflag
    finalXtraj = figure(1);
    view([60,20]);
    grid on
    axis square
    axis vis3d
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title({'DCP xtraj:';'DCP Desired Path and AT40GW Commanded Path'})
    hold on
    for n = 1:size(xtraj,1)
        scatter3(xtraj(n).at40(1:plotres:end,1), xtraj(n).at40(1:plotres:end,2), xtraj(n).at40(1:plotres:end,3), 10.*(xtraj(n).tool(1:plotres:end,3)+0.1), hsv2rgb(xtraj(n).tool(1:plotres:end,1:3)));
        scatter3(xtraj(n).dcp(1:plotres:end,1), xtraj(n).dcp(1:plotres:end,2), xtraj(n).dcp(1:plotres:end,3), 1, 'k');
        drawnow;
        pause(0.2);
    end
    hold off
end

%% Clear unneeded variables
disp('Waypoints successfully converted to xtraj!');
curvars = {curvars{:},'xtraj'};
clearvars('-except', curvars{:});
clear curvars;
close([1]);
