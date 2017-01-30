% SMOOTHCARTSEGTRAJ Creates a time-parametrized trajectory with smooth
% transitions from Cartesian waypoints

%{
smoothcartsegtraj.m
Julian Leland, MIT Media Lab, 2017-01-23
Originally written by Veevee Cai, MIT Media Lab, 2016

This function takes a series of ordered waypoints in a cell format and
converts them to a time-parametrized trajectory with smoothing between
waypoints.

%}

%% Setup
% Move parameters
dt = 0.01; % Desired timestep, seconds.
tacc = 0.01; % Desired Cartesian acceleration time (mm/s^2)
spd = 100; % Desired Cartesian velocity, (mm/s)

% Flags
simflag = 0; % Flag to indicate whether to use default home position or measure home position of vehicle.
% 1 = use default home. 0 = measure current home

rel = 1; % Flag for relative vs. absolute move. 
% 1 = 0,0,0 in image is set at endpoint of DCP. 0 = [0,0,0] in image is
% set at origin of DCP

robot = config_dcpv2;

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

%% Set up waypts if cell vs. matrix
if iscell(waypts)
    nsegs = length(waypts);
else
    nsegs = 1;
    temp_waypts = waypts;
    waypts = {};
    waypts{1} = temp_waypts;
end

%% Make it a relative move
if rel
    x0 = joint2cart_at40gw(q0);
    offset = x0 - waypts{1}(1,1:3);
    for n = 1:nsegs
        waypts{n} = bsxfun(@plus, waypts{n}, [offset,0,0]);
    end
end

%% Trajectories
% Setup trajectories for each segment (blend later)
% Can pre-blend here
% Nx4xM, N

qrawtrajs = {};
qdrawtrajs = {};
segts = {};
lightmode = {};
usedjoints = logical([1 0 1 1]);
i = 1;

for n = 1:nsegs
    lightmode{1,i} = 1;
    disp(['Pathing segment ', num2str(n),'...']);
    
    % Compute time estimates along segments
    [xtraj, xdtraj] = mstraj2(waypts{n}(2:end,1:3),spd,[],waypts{n}(1,1:3),dt,tacc);
    ts = (0:(size(xtraj,1)-1)) .* dt;
    
    qtraj = ikine_at40gw(xtraj, q0, usedjoints);
    qdtraj = cartvel2jointvel_at40gw(qtraj, xdtraj, usedjoints);
    qrawtraj = joint2raw_at40gw(robot, qtraj);
    qdrawtraj = jointvel2rawvel_at40gw(robot, qtraj, qdtraj);

    qrawtrajs = [qrawtrajs;qrawtraj];
    qdrawtrajs = [qdrawtrajs;qdrawtraj];
    segts = [segts;ts];
    q0 = qtraj(end,:);
    i = i + 1;
    if n < nsegs && any(waypts{n}(end,1:3) ~= waypts{n+1}(1,1:3))
        disp(['Pathing transition segment ', num2str(n),'...']);
        lightmode{1,i} = 0;
        
        % Get the parameterized trajectory
        xf = waypts{n+1}(1,1:3);
        x0 = waypts{n}(end,1:3);
        xdir = (xf-x0)./norm(xf-x0);
        [xt, xdt, xddt, ts] = lspb3(0,norm(xf-x0),spd,tacc,dt);

        % Transform into useable trajectories
        xtraj = bsxfun(@times, xt, xdir);
        xtraj = bsxfun(@plus, xtraj, x0);
        xdtraj = bsxfun(@times, xdt, xdir);
        
        %[xtraj, xdtraj] = mstraj2(waypts{n+1}(1,1:3),spd,[],waypts{n}(end,1:3),dt,tacc);
        %ts = (0:(size(xtraj,1)-1)) .* dt;
        
        qtraj = ikine_at40gw(xtraj, q0, usedjoints);
        q0 = qtraj(end,:);
        qdtraj = cartvel2jointvel_at40gw(qtraj, xdtraj, usedjoints);
        qrawtraj = joint2raw_at40gw(robot, qtraj);
        qdrawtraj = jointvel2rawvel_at40gw(robot, qtraj, qdtraj);

        qrawtrajs = [qrawtrajs;qrawtraj];
        qdrawtrajs = [qdrawtrajs;qdrawtraj];
        segts = [segts;ts];
        i = i + 1;
    end
end

% Check for exceeded joint positions
exceeded = checkjointlimits_at40gw(robot, cell2mat(qrawtrajs));
if any(exceeded(:))
    disp(['WARNING Joints exceeded limits in trajectory, joints are: ' num2str(find(any(exceeded,1)))]);
end

% Reset nsegs to be correct length
nsegs = length(qrawtrajs);

%% Visualize path

figure;
hold on;
axis equal;
grid on;
view([60,20]);

for m = 1:length(qrawtrajs)
    disp(m)
    for n = 1:100:length(qrawtrajs{m})
        xc = joint2cart_at40gw(raw2joint_at40gw(robot,qrawtrajs{m}));
        if lightmode{m}
            scatter3(xc(n,1),xc(n,2),xc(n,3),'b');
            drawnow;
        else
            scatter3(xc(n,1),xc(n,2),xc(n,3),'r');
            drawnow;
        end
    end
end
xlabel('x');

%{
joint2check = 4;
je = joint2check;
qm = cell2mat(qrawtrajs);
xm = joint2cart_at40gw(raw2joint_at40gw(robot,qm));
figure;
hold on;
axis equal;
scatter3(xm(~exceeded(:,je),1),xm(~exceeded(:,je),2),xm(~exceeded(:,je),3),'b');
scatter3(xm(exceeded(:,je),1),xm(exceeded(:,je),2),xm(exceeded(:,je),3),'r');
%}