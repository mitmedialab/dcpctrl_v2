% PATHVIZ Simple path vizualizer for DCPv.2


% INPUTS (expects in the workspace):
%   qrawtrajs - cells containing segments to follow
%   qdrawtrajs - cells containing segment raw joint velocities
%   dt - period for controller loop
%
% SETTINGS:
%   simflag - 1 enable simflaglation, 0 enable real robot interface
%   logflag - 1 enable logging, 0 disable logging
%   vizflagflag - 1 show visualization during print, 0 hide it
%   tol - (mm) tolerance for end of a segment

% Setup
vizflag = 1;
simflag = 1;
wait4settle = 1; % Waits at end of each segment to stop moving
tol = 50; % mm

usedjoints = logical([1 0 1 1]);

global stopflag;
stopflag = 0;

%% Setup visualization
if vizflag
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
    
    q0 = raw2joint_at40gw(robot, q0raw)
    xttraj = joint2cart_at40gw(raw2joint_at40gw(robot, cell2mat(qrawtrajs)));
    arm0 = joint2arm_at40gw(q0);
    seg0 = joint2cart_at40gw(raw2joint_at40gw(robot, qrawtrajs{1}));
    xtrav = xttraj(1,:);

    figure;
    hold on;
    grid on;
    view([60,20]);
    plot3(xttraj(:,1),xttraj(:,2),xttraj(:,3),'c');
    harm = plot3(arm0(:,1),arm0(:,2),arm0(:,3),'g','LineWidth',5);
    hdes = plot3(xttraj(1,1),xttraj(1,2),xttraj(1,3),'mx','MarkerSize',13,'LineWidth',2);
    hseg = plot3(seg0(:,1),seg0(:,2),seg0(:,3),'b','LineWidth',3);
    htrav = plot3(xtrav(1,1),xtrav(1,2),xtrav(1,3),'r.');
    hpause = uicontrol('Style', 'PushButton', 'String', 'Pause','Callback','pauseprint(1)');
    axis equal;
    hold off;
end

%% Setup FFPD Controller gains

Kps = [robot.Joint.Kp];
Kds = [robot.Joint.Kd];
Kis = [robot.Joint.Ki];
Kvs = [robot.Joint.Kv];
GainInvs = [robot.Joint.GainInv];
movethresh = [robot.Joint.moveThresh];

rawlimstructs = [robot.Joint.PosLim];
rawmaxlim = [rawlimstructs.Max];
rawminlim = [rawlimstructs.Min];
% WARNING: handle this corner case more elegantly somehow
[rawmaxlim(2), rawminlim(2)] = deal(rawminlim(2),rawmaxlim(2));

pwmlimstructs = [robot.Joint.PWMLim];
pwmmax = [pwmlimstructs.Max];
pwmdbmax = [pwmlimstructs.DBUpper];
pwmdbmin = [pwmlimstructs.DBLower];
pwmmin = [pwmlimstructs.Min];

%% Reset the segments
segstart = 1;

% Can use this with move2raw to auto position
qfraw = qrawtrajs{segstart}(1,:);

%% Follow trajectory
dispstat('','init');
for segi = segstart:nsegs
    
    if stopflag == 1
        disp(['Just finished segment ' num2str(segi-1) ' next segment is ' num2str(segi)]);
        segstart = segi;
        qfraw = qrawtrajs{segi}(1,:);
        stopflag = 0;
        set(hpause,'Enable','on');
        break;
    end
    
    % Setup for segment
    qrawtraj = qrawtrajs{segi};
    qdrawtraj = qdrawtrajs{segi};
    ts = segts{segi};
    tf = ts(end)+dt;
    npoints = size(qrawtraj,1);
    %lightcontrol(lightmode{segi})
    ti = 1;
    qdrawcomm = zeros(1,4);
    tprev = 0;
    qrawdesprev = qrawtraj(1,:);
    qdrawdesprev = qdrawtraj(1,:);
    errprev = 0;
    
    % Reset Kie for each segment (should we do this, or allow it to
    % accumulate)
    Kie = zeros(1,4);
    
    xf = joint2cart_at40gw((raw2joint_at40gw(robot, qrawtraj(end,:))));
    
    qrawprev = qrawtraj(1,:);
    
    logs = {};
    logi = 1;
    
    % Update visualization (segment only)
    if vizflag
        segt = joint2cart_at40gw(raw2joint_at40gw(robot, qrawtraj));
        set(hseg,'xdata',segt(:,1),'ydata',segt(:,2),'zdata',segt(:,3));
        drawnow;
    end
    
    % Follow the trajectory
    tic;
    t = 0;
    while true
        
        if stopflag
            set(hpause,'Enable','off');
        end
        
        t = toc;

        qraw = qrawprev + qdrawcomm.*(t-tprev);
        
        if ~wait4settle && segi == nsegs && t > tf && all(abs(qraw(usedjoints)-qrawtrajs{end}(end,usedjoints)) < movethresh(usedjoints))
            disp(['Just finished trajectory.']);
            break;
        elseif wait4settle && t > tf && all(abs(qraw(usedjoints)-qrawtraj(end,usedjoints)) < movethresh(usedjoints)) 
            disp(['Just finished segment ' num2str(segi)]);
            break;
        end
        
        % Time-consistent controls
        if (t-tprev) < dt
            continue;
        end
        
        % Setup for iteration
        ti = min(ceil(t / dt), size(qrawtraj,1));
        talpha = mod(t, dt) / dt;
        if ti ~= 1
            qrawdesprev = qrawtraj(ti-1, :);
            qdrawdesprev = qdrawtraj(ti-1, :);
        end
        qrawdes = (1-talpha) .* qrawdesprev + talpha .* qrawtraj(ti, :);
        qdrawdes = (1-talpha) .* qdrawdesprev + talpha .* qdrawtraj(ti, :);
        
        if t > tf && segi < nsegs && ~wait4settle
            break;
        elseif t > tf
            qrawdes = qrawtraj(end, :);
            qdrawdes = qdrawtraj(end, :);
        end
        
        % Compute errors
        err = qrawdes - qraw;
        derr = err - errprev;
        dmeaserr = qraw - qrawprev;
        
        % Compute command
        Kffv = Kvs .* qdrawdes;
        Kpe = Kps .* err;
        Kde = - Kds .* dmeaserr; % This must be negative
        Kie = Kis .* err + Kie;
        Kie = clamp(Kie, rawminlim, rawmaxlim); % clamp on raw or draw?
        %qrawcomm = clamp(qrawcomm, rawminlim, rawmaxlim);
        
        qdrawcomm = Kffv + Kpe + Kde + Kie;
    
        % Setup for next iteration
        tprev = t;
        errprev = err;
        qrawprev = qraw;
        
        % Update visualization
        if vizflag
            qv = raw2joint_at40gw(robot, qraw);
            armv = joint2arm_at40gw(qv);
            xdesv = joint2cart_at40gw(raw2joint_at40gw(robot, qrawdes));
            xtrav = [xtrav; joint2cart_at40gw(raw2joint_at40gw(robot, qraw))];

            set(htrav, 'xdata', xtrav(:,1), 'ydata', xtrav(:,2), 'zdata', xtrav(:,3));
            set(harm, 'xdata', armv(:,1), 'ydata', armv(:,2), 'zdata', armv(:,3));
            set(hdes, 'xdata', xdesv(1), 'ydata', xdesv(2), 'zdata', xdesv(3));
            drawnow;
 
        end
    end
end
