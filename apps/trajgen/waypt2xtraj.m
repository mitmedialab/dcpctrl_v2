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
spd = 250; % Cartesian velocity for trajectory, distance units/s
tacc = 50; % Cartesian acceleration time, distance units/s^2.

plotres = 10;

vizflag = 1;

%% Extract position and time from waypts and feed into waypts2carttraj
waypts_pos = {};
waypts_t = {};
xtraj_temp = {}; % Generate empty cell array to hold DCP xtraj

% Extract time and position from waypts
for n = 1:size(waypts,1)
    waypts_pos{n,1} = waypts{n}(:,2:4);
    waypts_t{n,1} = waypts{n}(:,2:4);
end

% Add dummy vectors to waypts
for n = 1:size(waypts_pos,1)
    % This creates dummy trajectories for the AT40, KUKA , tool and enable
    waypts_pos(n,2:5) = {[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0]};
end

% New struct format
xtraj_temp = {}; % Generate empty cell array to hold DCP xtraj
for n = 1:size(waypts_pos,1)
     xtraj_temp(n,:) = waypts2carttraj(waypts_pos(n,:),dt,spd,tacc);
end

%% Re-insert correct tool trajectory
% For now, we're just inserting the first set of tool commands for each
% segment for the entire segment. Eventually, we will map tool commands
% from the waypoints to xtraj_temp by time

for n = 1:size(xtraj_temp,1)
    xtraj_temp{n,5} = repmat(waypts{n}(1,5:10),size(xtraj_temp{n,5},1),1);
end

%% Set enable trajectories appropriately
% Eventually, this will be done wherever we sort tasks between the AT40GW
% and the KUKA

for n = 1:size(xtraj_temp,1)
    xtraj_temp{n,6} = repmat([1 0 1],size(xtraj_temp{n,6},1),1);
end

%% Create real AT40GW task trajectory
% Just copy the DCP move to the AT40GW, since the AT40 is the only part
% that is moving

xtraj_temp(:,3) = xtraj_temp(:,2);

%% Convert trajectory to structure

xtraj = struct('t',xtraj_temp(:,1),'dcp',xtraj_temp(:,2),'at40',xtraj_temp(:,3),'kuka',xtraj_temp(:,4),'tool',xtraj_temp(:,5),'en',xtraj_temp(:,6));

%% OPTIONAL: Check trajectory to make sure it makes sense
if vizflag
    finalXtraj = figure(1);
    view([60,20]);
    grid on
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

%% Clear unneeded variables
disp('Waypoints successfully converted to xtraj!');
curvars = {curvars{:},'xtraj'};
clearvars('-except', curvars{:});
clear curvars;
close([1]);