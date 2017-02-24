% SLXTOOLPATHCONVERT Converts MATLAB trajectories to Simulink-compatible
% trajectories.

%{
slxtoolpathconvert.m
Julian Leland, MIT Media Lab, 2017-01-11

This function takes toolpaths generated using dcpctrl_v1 techniques and
transforms them into a form usable under dcpctrl_v2.

%}

%% Set up temp variables for timeseries
q_temp = [];
qd_temp = [];
qraw_temp = [];
qdraw_temp = [];
tool_temp = [];

%% Change toolpath name & get number of tool channels
maxtoolchans = 6; % Maximum number of tool channels supported - creates dummy channels

toolmode = tooltrajs;
toolchans = size(tooltrajs{1},2); % Assume all tool trajectories are the same size (they should be!)
tool_complete = (size(tooltrajs{1},1) > 1); % If this is greater than one, then we assume the tool trajectory is fully specified.
tool_vect = []; % Empty tool vector for each run

%% Collect trajs and measure total trajectory length
trajlen = 0;
for n = 1:length(qrawtrajs)
    trajlen = trajlen + sum(size(qrawtrajs{n},1));
    q_temp = [q_temp;qtrajs{n}];
    qd_temp = [qd_temp;qdtrajs{n}];
    qraw_temp = [qraw_temp;qrawtrajs{n}];
    qdraw_temp = [qdraw_temp;qdrawtrajs{n}];
    for m = 1:toolchans
        if tool_complete
            tool_vect = [tool_vect,toolmode{n}(:,m)];
        else
            tool_vect = [tool_vect,ones(size(qrawtrajs{n},1),1)*toolmode{n}(1,m)];
        end
    end
    if toolchans < maxtoolchans
        for m = toolchans+1:maxtoolchans
            if tool_complete
                tool_vect = [tool_vect,zeros(size(qrawtrajs{n},1),1)];
            else
                tool_vect = [tool_vect,zeros(size(qrawtrajs{n},1),1)];
            end
        end
    end
    tool_temp = [tool_temp;tool_vect];
    tool_vect = [];
end

%% Create time vector
t_temp = linspace(0,trajlen*dt,trajlen)';

%% Create timeseries collection
qseries = timeseries(q_temp,t_temp,'Name','q');
qdseries = timeseries(qd_temp,t_temp,'Name','qd');
qrawseries = timeseries(qraw_temp,t_temp,'Name','qraw');
qdrawseries = timeseries(qdraw_temp,t_temp,'Name','qdraw');
toolseries = timeseries(tool_temp,t_temp,'Name','tool');
%traj = tscollection({qrawseries,qdrawseries},'Name','rawtraj');
traj.q = qseries;
traj.qd = qdseries;
traj.qraw = qrawseries;
traj.qdraw = qdrawseries;
traj.tool = toolseries;