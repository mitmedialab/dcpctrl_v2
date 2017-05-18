% QTRAJ2SLXTRAJ Converts MATLAB trajectories to Simulink-compatible
% trajectories.

%{
qtraj2slxtraj.m
Julian Leland, MIT Media Lab, 2017-02-25

This function takes qtraj generated in compliance with Trajectory Input 
Standard Definition (02-2017) and converts them into a form usable in
Simulink. It also checks for trajectory validity, and plots the trajectory
using old-school plot methods.

%}

%% Get list of all variables currently in workspace
curvars = who; % Get current variables

%% Setup
vizflag = 1; % Flag to show or hide visualizations

%% Set up temp variables for timeseries
traj_temp = {};

t_temp = [];
dcp_x_temp = [];
dcp_xd_temp = [];
at40_q_temp = [];
at40_qd_temp = [];
% at40_qraw_temp = []; % Not using qraw/qdraw currently
% at40_qdraw_temp = [];
kuka_x_temp = [];
kuka_xd_temp = [];
kuka_q_temp = [];
kuka_qd_temp = [];
tool_temp = [];
en_temp = [];

%% Collect trajs and measure total trajectory length
trajlen = 0;

% Insert first segment in trajectory
trajlen = sum(size(qtraj(1).t,1));
t_temp = qtraj(1).t;
dcp_x_temp = qtraj(1).dcp(:,1:3);
dcp_xd_temp = qtraj(1).dcp(:,4:6);
at40_q_temp = qtraj(1).at40(:,1:4);
at40_qd_temp = qtraj(1).at40(:,5:8);
% at40_qraw_temp = []; % Not using qraw/qdraw currently
% at40_qdraw_temp = [];
kuka_x_temp = qtraj(1).kuka_x(:,1:3);
kuka_xd_temp = qtraj(1).kuka_x(:,4:6);
kuka_q_temp = qtraj(1).kuka_q(:,1:6);
kuka_qd_temp = qtraj(1).kuka_q(:,7:12);
tool_temp = qtraj(1).tool;
en_temp = qtraj(1).en;

for n = 2:size(qtraj,1)
    trajlen = trajlen + sum(size(qtraj(n).t,1)); % Total number of points in trajectory
    t_temp = [t_temp;(qtraj(n).t + t_temp(end))];
    dcp_x_temp = [dcp_x_temp; qtraj(n).dcp(:,1:3)];
    dcp_xd_temp = [dcp_xd_temp; qtraj(n).dcp(:,4:6)];
    at40_q_temp = [at40_q_temp; qtraj(n).at40(:,1:4)];
    at40_qd_temp = [at40_qd_temp; qtraj(n).at40(:,5:8)];
    % at40_qraw_temp = []; % Not using qraw/qdraw currently
    % at40_qdraw_temp = [];
    kuka_x_temp = [kuka_x_temp; qtraj(n).kuka_x(:,1:3)];
    kuka_xd_temp = [kuka_xd_temp; qtraj(n).kuka_x(:,4:6)];
    kuka_q_temp = [kuka_q_temp; qtraj(n).kuka_q(:,1:6)];
    kuka_qd_temp = [kuka_qd_temp; qtraj(n).kuka_q(:,7:12)];
    tool_temp = [tool_temp; qtraj(n).tool];
    en_temp = [en_temp; qtraj(n).en];
end

traj_temp = {t_temp,dcp_x_temp,dcp_xd_temp,at40_q_temp,at40_qd_temp,kuka_x_temp,kuka_xd_temp,kuka_q_temp,kuka_qd_temp,tool_temp,en_temp};

%% Verify that all series are the correct length
sizes = zeros(size(traj_temp));
for n = 1:size(traj_temp,2)
    sizes(n) = size(traj_temp{n},1);
end

if any(sizes ~= trajlen)
    disp('One trajectory is not full. Please check and try again.');
    return;
end

%% Create timeseries representations 

dcp_x = timeseries(dcp_x_temp,t_temp,'Name','dcp_x');
dcp_xd = timeseries(dcp_xd_temp,t_temp,'Name','dcp_xd');

% at40_qraw = timeseries(qraw_temp,t_temp,'Name','qraw');
% at40_qdraw = timeseries(qdraw_temp,t_temp,'Name','qdraw');

at40_q = timeseries(at40_q_temp,t_temp,'Name','at40_q');
at40_qd = timeseries(at40_qd_temp,t_temp,'Name','at40_qd');

kuka_x = timeseries(kuka_x_temp,t_temp,'Name','kuka_x');
kuka_xd = timeseries(kuka_xd_temp,t_temp,'Name','kuka_xd');

kuka_q = timeseries(kuka_q_temp,t_temp,'Name','kuka_q');
kuka_qd = timeseries(kuka_qd_temp,t_temp,'Name','kuka_qd');

tool = timeseries(tool_temp,t_temp,'Name','tool');

en = timeseries(en_temp,t_temp,'Name','tool');

%% Generate old-style trajectories and visualize
% Super hacky, not reliable - replace later with better simulator.

if vizflag
    qrawtrajs = {j2rfcn_at40gw(at40_q_temp(:,1),at40_q_temp(:,2),at40_q_temp(:,3),at40_q_temp(:,4))};
    qdrawtrajs = {jointvel2rawvel_at40gw(robot,at40_q_temp(:,1:4),at40_qd_temp(:,1:4))};
    dt = max(diff(t_temp));
    pathviz(robot, qrawtrajs, qdrawtrajs, t_temp, dt);
end

%% Clear unneeded variables
disp('Simulink trajectory created!');
curvars = {curvars{:},'qtraj','dcp_x','dcp_xd','at40_q','at40_qd','kuka_x','kuka_xd','kuka_q','kuka_qd','tool','en'};
clearvars('-except', curvars{:});
clear curvars;