% SLXTOOLPATHCONVERT Converts MATLAB trajectories to Simulink-compatible
% trajectories.

%{
slxtoolpathconvert.m
Julian Leland, MIT Media Lab, 2017-01-11

This function takes toolpaths generated using dcpctrl_v1 techniques and
transforms them into a form usable under dcpctrl_v2.

%}

%% Set up temp variables for timeseries
qraw_temp = [];
qdraw_temp = [];
tool_temp = [];

%% Change toolpath name
toolmode = lightmode;

%% Collect trajs and measure total trajectory length
trajlen = 0;
for n = 1:length(qrawtrajs)
    trajlen = trajlen + sum(size(qrawtrajs{n},1));
    qraw_temp = [qraw_temp;qrawtrajs{n}];
    qdraw_temp = [qdraw_temp;qdrawtrajs{n}];
    tool_temp = [tool_temp;ones(size(qrawtrajs{n},1),1)*toolmode{n}];
end

%% Create time vector
t_temp = linspace(0,trajlen*dt,trajlen);

%% Create timeseries collection
qrawseries = timeseries(qraw_temp,t_temp,'Name','qraw');
qdrawseries = timeseries(qdraw_temp,t_temp,'Name','qdraw');
toolseries = timeseries(tool_temp,t_temp,'Name','tool');
%traj = tscollection({qrawseries,qdrawseries},'Name','rawtraj');
traj.qraw = qrawseries;
traj.qdraw = qdrawseries;
traj.tool = toolseries;