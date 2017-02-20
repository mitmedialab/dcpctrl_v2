% ISO9283POSETRAJ Generate trajectory for ISO 9283 pose repeatability
% testing

%{
iso9283posetraj
Julian Leland, MIT Media Lab
2017-02-12 (originally written 2016-07

This function generates a complete trajectory (NOT waypoints) for the ISO
9283 pose repeatability test. This version also generates a tool trajectory
for light painting, with a different color corresponding to each leg of the
trajectory.

%}

%% Define test parameters
dt = 0.1;
tacc = 1;
spd = 200; % mm/s (cartesian)

len = 2500; % Length of cube size, mm
numreps = 1; % Number of times to move through trajectory
cornerdelay = 7; % Delay (in sec) at each corner of a trajectory
enddelay = 15; % Delay (in sec) at end of each run.

simflag = 1; % Flag to simulate at default home position (1), or to run at current position (0)
robot = config_dcpv2;

%% Generate cube
% Robot endpoint is assumed to start at center of cube - we will need to
% shift coordinates to here.
C0 = [0,0,0]; % Center of cube
C1 = [len/2,len/2,len/2];
C2 = [len/2,-len/2,len/2];
C3 = [-len/2,-len/2,len/2]; 
C4 = [-len/2,len/2,len/2]; 
C5 = [len/2,len/2,-len/2]; 
C6 = [len/2,-len/2,-len/2];
C7 = [-len/2,-len/2,-len/2];
C8 = [-len/2,len/2,-len/2];
c = [C0;C1;C2;C3;C4;C5;C6;C7;C8];
diaglen = norm([len len len]);

%% Select which corners of cube should be used & generate plane
coff = 0.1*diaglen; % Calculate offset from corners to create P points
cl = sqrt((coff^2)/3);
plntype = 'a'; % Available choices: a,b,c,d,xz,yz,xy. We will implement other plane types later.
switch plntype
    case 'a'
        xtraj_0 = [C0; ...
            C1+[-cl,-cl,-cl];...
            C2+[-cl,cl,-cl];...
            C7+[cl,cl,cl];...
            C8+[cl,-cl,cl];...
            C0];  
end

%% Get home position of robot & shift points to there
if simflag
    q0raw = robot.HomeRawPos;
else
    open('getrawpos_slx.slx'); % Open SLX
    simTime = get_param('getrawpos_slx','StopTime'); % Measure currently set simulation time
    set_param('getrawpos_slx', 'SimulationCommand', 'start'); % Run SLX
    while strcmp(get_param('getrawpos_slx','SimulationStatus'),'external')
        % Wait while SLX executes so that q0raw shows up.
        disp('Measuring robot current position - please wait.');
        pause(str2num(simTime)+0.1); % Wait until just after simulation has stopped.
    end
    close_system('getrawpos_slx');
end

q0 = raw2joint_at40gw(robot, q0raw);
x0 = joint2cart_at40gw(q0); 

xtraj_t = bsxfun(@plus, xtraj_0, x0);
xtraj_t = [xtraj_t,ones(size(xtraj_t,1),1)];

%% Replicate as many times as specified
xtraj = xtraj_t'; % Transpose for speed
for n = 2:numreps
    temp = [bsxfun(@times,xtraj_t,[1,1,1,n])]';
    xtraj = [xtraj,temp];
end
waypts = xtraj';

% %% Add extra delay segments.
% for n = 1:length(waypts)-1
%     if waypts(n,4) ~= waypts(n+1,4)
        

%% Move from Cartesian to joint space.
qwaypts = ikine_at40gw(waypts(:,1:3), q0, [1 0 1 1]);
qwaypts = [qwaypts,waypts(:,4)];
nwaypts = size(qwaypts,1);
nsegs = nwaypts - 1;
qrawtrajs = cell(nsegs,1);
qdrawtrajs = cell(nsegs,1);
qtrajs = cell(nsegs,1);
qdtrajs = cell(nsegs,1);
%pwmtrajs = cell(nsegs,1);
tooltrajs = cell(nsegs,2);
segts = cell(nsegs,1); % relative timestamps
absts = cell(nsegs,1); % absolute timetamps
mtypes = cell(nsegs,1);
ttf = 0;
usedjoints = logical([1 0 1 1]);
colors = [0 35 50 240 285]; % Hue values for light output

dispstat('','init');
for i = 1:nsegs
    % Get endpoint information
    %dispstat(sprintf('Generating traj for segment: %d/%d',i,nsegs));
    fprintf('Generating traj for segment: %d/%d\n',i,nsegs)
    
    % Set color for this segment
    tooltraj = [1,colors(1,mod(i-1,5)+1)];
    
    if qwaypts(i,5) == qwaypts(i+1,5)
        % We are moving
        q0 = qwaypts(i,1:4);
        qf = qwaypts(i+1,1:4);
        mtype = 1;
        x0 = joint2cart_at40gw(q0);
        xf = joint2cart_at40gw(qf);
        xdir = (xf-x0)./norm(xf-x0);

        % Get the parameterized trajectory
        [xt, xdt, xddt, ts] = lspb3(0,norm(xf-x0),spd,tacc,dt);

        % Transform into useable trajectories
        xtraj = bsxfun(@times, xt, xdir);
        xtraj = bsxfun(@plus, xtraj, x0);
        xdtraj = bsxfun(@times, xdt, xdir);

        qtraj = ikine_at40gw(xtraj, q0, usedjoints);
        qrawtraj = joint2raw_at40gw(robot, qtraj);
        qdtraj = cartvel2jointvel_at40gw(qtraj, xdtraj, usedjoints);
        qdrawtraj = jointvel2rawvel_at40gw(robot, qtraj, qdtraj);
        %pwmtraj = rawvel2pwm_at40gw(robot, qdrawtraj);
        
        % Add additional points for dwell at end of move
        tscount = cornerdelay/dt; % Number of timesteps of zero motion that we need to inject
        qrawtraj = [qrawtraj;bsxfun(@times,ones(tscount,4),qrawtraj(end,:))];
        qdrawtraj = [qdrawtraj;zeros(tscount,4)];
        qtraj = [qtraj;bsxfun(@times,ones(tscount,4),qtraj(end,:))];
        qdtraj = [qdtraj;zeros(tscount,4)];
        %pwmtraj = [pwmtraj;ones(tscount,4).*80000];
        ts = [ts;[ts(end):dt:(ts(end)+dt*tscount)]'];
        
        % Check that velocities don't exceed max joint velocities at any
        % point in trajectory - break if so.
        %{
        exceeded = checkjointvellimits_at40gw(robot, qdrawtraj);
        if any(exceeded(:))
            fprintf(['WARNING Joints exceeded velocity limits in trajectory, joints are: ' num2str(find(any(exceeded,1))),'\n']);
            fprintf(['Trajectory segment: ', num2str(i),'\n']);
        end
        %}

        % Save all the trajectories
        mtypes{i} = mtype;
        qrawtrajs{i} = qrawtraj;
        qdrawtrajs{i} = qdrawtraj;
        qtrajs{i} = qtraj;
        qdtrajs{i} = qdtraj;
        %pwmtrajs{i} = pwmtraj;
        tooltrajs{i} = tooltraj;
        segts{i} = ts;
        absts{i} = ts + ttf;
        ttf = ttf + ts(end);
        
    else
        % We are at the end of a diamond. Generate static dwell points to
        % stop.
        mtype = 1; % Setting mtype to 1 for now - don't think it matters for now
        tscount = enddelay/dt; % Number of timesteps of zero motion that we need to inject
        qrawtraj = bsxfun(@times,ones(tscount,4),qrawtrajs{i-1,1}(end,:));
        qdrawtraj = zeros(tscount,4);
        qtraj = bsxfun(@times,ones(tscount,4),qtrajs{i-1,1}(end,:));
        qdtraj = zeros(tscount,4);
        %pwmtraj = ones(tscount,4).*80000;
        ts = [0:dt:enddelay]';
        mtypes{i} = mtype;
        qrawtrajs{i} = qrawtraj;
        qdrawtrajs{i} = qdrawtraj;
        qtrajs{i} = qtraj;
        qdtrajs{i} = qdtraj;
        %pwmtrajs{i} = pwmtraj;
        tooltrajs{i} = tooltraj;
        segts{i} = ts;
        absts{i} = ts + ttf;
        ttf = ttf + ts(end);
    end
end