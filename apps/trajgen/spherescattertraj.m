% SPHERESCATTERTRAJ Generate trajectory for set of spheres located
% throughout DCP workspace

%{
spherescattertraj
Julian Leland, MIT Media Lab
2017-07-08

This function generates a trajectory to create a "forest" of spheres
throughout the DCP's workspace. The AT40GW moves between a series of
positions in space. At each of these positions, the KUKA transcribes a
spherical, helical path of variable size.

%}

%% Get list of all variables currently in workspace
curvars = who; % Get current variables

%% Define test parameters
dt = 0.1;
tacc = 1;
spd = 100; % mm/s (cartesian)

kuka_spd = 150; % mm/s
kuka_tacc = 0.1;

vizflag = 1; % Flag to show images/simulations
plotres = 10; % Display every plotres'th point when doing large scatter plots

%% Define positions and sphere sizes
% Super simplified input format, [xrel yrel zrel radius]; 
% In the future, this could be automatically imported.

spheretraj = [0 1000 -1000 100;
              0 0     0    50;
              0 -1000 1000 150];

at40_xtraj = spheretraj(:,1:3);
kuka_radtraj = spheretraj(:,4);

%% Create waypoints for AT40GW moves
waypts = {};

% Add starting position (0,0,0) to at40_xtraj
at40_xtraj = [0,0,0;at40_xtraj];

% Add initial xtraj to waypots
for n = 1:size(at40_xtraj,1)-1
    waypts{n,1} = [at40_xtraj(n,:);at40_xtraj(n+1,:)];
end
%% Create trajectories for AT40GW moves
% Add dummy vectors to waypts
for n = 1:size(waypts,1)
    % This creates dummy trajectories for the AT40, KUKA , tool and enable
    waypts(n,2:5) = {[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0]};
end

% New struct format
at40traj_temp = {}; % Generate empty cell array to hold DCP xtraj
for n = 1:size(waypts,1)
     at40traj_temp(n,:) = waypts2carttraj(waypts(n,:),dt,spd,tacc);
end

% Swap AT40GW segments to appropriate location
at40traj_temp_at40 = at40traj_temp(:,2);
at40traj_temp_dcp = at40traj_temp(:,3);
at40traj_temp(:,3) = at40traj_temp_at40;
at40traj_temp(:,2) = at40traj_temp_dcp;
%% Create spiral trajectories for KUKA
kukatraj_temp = {}; % Create empty cell array to hold KUKA and light trajectories

for n = 1:size(kuka_radtraj,1)
    curRad = kuka_radtraj(n); % Get current radius
    % Method for generating spiral:
    % 1: Generate trajectory from start point down to base of sphere
    waypts_start = {[0,0,0; 0,0,-curRad]};
    for m = 1:size(waypts_start,1)
        waypts_start(m,2:5) = {[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0]};
    end
    
    % 2: Generate spherical spiral path
    % Parametric spiral: https://math.stackexchange.com/questions/140541/finding-parametric-curves-on-a-sphere
    a = 10; % Define a coefficient (number of revolutions
    % Note that even though we're defining a t vector here, we just use
    % this to generate waypoints - we still use waypts2carttraj to ensure
    % that the path is executed at constant Cartesian speed.
    t_spiraltemp = linspace(-1,1,1*curRad)'; % Arbitrarily multiplying number of points by curRad so it scales
    x_spiraltemp = sqrt(1 - t_spiraltemp.^2).*cos(a*pi.*t_spiraltemp);
    y_spiraltemp = sqrt(1 - t_spiraltemp.^2).*sin(a*pi.*t_spiraltemp);
    z_spiraltemp = t_spiraltemp;
    waypts_spiraltemp = {[x_spiraltemp,y_spiraltemp,z_spiraltemp].*curRad}; % Scale by curRad here.
    for m = 1:size(waypts_spiraltemp,1)
        waypts_spiraltemp(m,2:5) = {[0,0,0,0,0,0],[0,0,0,0,0,0],[0.5,0,1,0,0,0],[0,0,0]};
    end
    
    % 3: Generate trajectory from end point down to top of sphere
    waypts_end = {[0,0,curRad; 0,0,0]};
    for m = 1:size(waypts_end,1)
        waypts_end(m,2:5) = {[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0]};
    end
    
    % 4: Generate trajectory
    waypts_spiral = [waypts_start;waypts_spiraltemp;waypts_end];
    xtraj_spiral = {};
    for m = 1:size(waypts_spiral,1)
        xtraj_segment = waypts2carttraj(waypts_spiral(m,:),dt,kuka_spd,kuka_tacc);
        xtraj_spiral = [xtraj_spiral;xtraj_segment];
    end
    
    % Swap KUKA segments to appropriate locations
    xtraj_spiral_kuka = xtraj_spiral(:,2);
    xtraj_spiral_dcp = xtraj_spiral(:,4);
    xtraj_spiral(:,2) = xtraj_spiral_dcp;
    xtraj_spiral(:,4) = xtraj_spiral_kuka;
    
    kukatraj_temp{n,1} = xtraj_spiral;
end
%% Intersperse KUKA trajectories and set AT40 trajectories to previous values
xtraj_comb = {}; % Create empty cell array to hold trajectories
for n = 1:size(at40traj_temp,1)
    at40_temp = at40traj_temp(n,:); % Pick AT40GW trajectory
    at40_endpos = at40_temp{3}(end,:); % Get final position of AT40GW for this segment
    kuka_temp = [kukatraj_temp{n}]; % Get KUKA trajectory segments
    for m = 1:size(kuka_temp,1)
        kuka_temp{m,3} = ones(size(kuka_temp{m,3})).*at40_endpos;
    end
    xtraj_comb = [xtraj_comb;at40_temp;kuka_temp];
end

%% Create full trajectory
xtraj = struct('t',xtraj_comb(:,1),'dcp',xtraj_comb(:,2),'at40',xtraj_comb(:,3),'kuka',xtraj_comb(:,4),'tool',xtraj_comb(:,5),'en',xtraj_comb(:,6));

%% Add KUKA and AT40 trajectories to get DCP trajectory (for visualization only)
for n = 1:size(xtraj,1)
    xtraj(n).dcp = xtraj(n).kuka+xtraj(n).at40;
end

%% Set enable trajectories correctly
for n = 1:size(xtraj,1)
    if sum(sum(xtraj(n).kuka)) ~= 0
       xtraj(n).en(:,1) = 0;
       xtraj(n).en(:,2) = 1;
       xtraj(n).en(:,3) = 1;
    else
       xtraj(n).en(:,1) = 1;
       xtraj(n).en(:,2) = 0;
       xtraj(n).en(:,3) = 0;
    end
end

%% OPTIONAL: Check trajectory to make sure it makes sense
if vizflag
    figure(8);
    view([60,20]);
    axis square
    axis vis3d
    hold on
    for n = 1:size(xtraj,1)
        if xtraj(n).en(:,3)
            scatter3(xtraj(n).dcp(1:plotres:end,1), xtraj(n).dcp(1:plotres:end,2), xtraj(n).dcp(1:plotres:end,3),10,'r');
        else
            scatter3(xtraj(n).dcp(1:plotres:end,1), xtraj(n).dcp(1:plotres:end,2), xtraj(n).dcp(1:plotres:end,3),5,'k');
        end
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
