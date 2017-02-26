% XTRAJ2QTRAJ.M Converts task-space (Cartesian) trajectory to joint space
% for DCP

%{

xtraj2qtraj.m
Julian Leland, MIT Media Lab
2017-02-18

Edited to comply with Trajectory Input Standard Definition (02-2017) on
2017-02-24

This function converts a task-space (Cartesian) trajectory to a
joint-space trajectory. It does this by:
1) Locating the trajectory appropriately in the workspace;
2) Identifying starting joint angles for the AT40GW at this location
(assuming the KUKA is at its default home position)
3) Solving inverse kinematics to convert from task space to joint space for
the AT40GW

Currently, this only operates for the AT40GW - however, it may in the
future work for the KUKA as well. In the meantime, a dummy joint-space
trajectory is added for the KUKA as well.

%}

%% Get list of all variables currently in workspace
curvars = who; % Get current variables

%% Setup 
vizflag = 1; % Flag to show or hide figures

% Options for trajectory location: 
% 'cur' = current location
% 'def' = default home location
% [X Y Z] = absolute position in workspace
trajtype = 'def';

usedjoints = logical([1 0 1 1]); % Mask vector for which joints to use on the AT40GW

%% Create qtraj from xtraj
qtraj = xtraj;

%% Locate trajectory in workspace

if isnumeric(trajtype) && all(size(trajtype) == [1 3])
    q0raw = ikine_at40gw(trajtype,robot.HomeRawPos,[1 0 1 1]);
else
    switch trajtype
        case 'cur'
            % This function *kinda* works for using Simulink to get the robot's
            % current joint position. It's not ideal, though.
            open('getrawpos_slx.slx'); % Open SLX
            simTime = get_param('getrawpos_slx','StopTime'); % Measure currently set simulation time
            set_param('getrawpos_slx', 'SimulationCommand', 'start'); % Run SLX
            while strcmp(get_param('getrawpos_slx','SimulationStatus'),'external')
                % Wait while SLX executes so that q0raw shows up.
                disp('Measuring robot current position - please wait.');
                pause(str2num(simTime)+0.1); % Wait until just after simulation has stopped.
            end
            close_system('getrawpos_slx');

        case 'def'
            q0raw = robot.HomeRawPos;

        otherwise
            disp('Invalid input. Exiting.');
    end
end

q0 = r2jfcn_at40gw(q0raw(1),q0raw(2),q0raw(3),q0raw(4));

disp(['Current robot endpoint position: [',num2str(q0),']']);

%% Offset DCP and AT40GW points to current location

disp('Ofsetting toolpath to current robot endpoint position...');
x0 = joint2cart_at40gw(q0); % Note - this assumes the KUKA is in its default configuration
% For now, these two offsets should be the same, but they might not always be.
offset_dcp = x0 - qtraj(1).dcp(1,1:3);
offset_at40 = x0 - qtraj(1).at40(1,1:3);
for n = 1:size(qtraj,1)
    % Add offset to both AT40GW and DCP vectors
    qtraj(n).dcp(:,1:3) = bsxfun(@plus, qtraj(n).dcp(:,1:3), offset_dcp);
    qtraj(n).at40(:,1:3) = bsxfun(@plus, qtraj(n).at40(:,1:3), offset_at40);
end
disp('Done!');

%% OPTIONAL: Check that points for trajectories still look correct
if vizflag
    figure(1);
    view([60,20]);
    axis square
    axis vis3d
    hold on
    for n = 1:size(qtraj,1)
        scatter3(qtraj(n).dcp(:,1),qtraj(n).dcp(:,2),qtraj(n).dcp(:,3));
        drawnow;
        pause(0.2);
    end
    hold off
end

%% Convert AT40GW points to joint space
for n = 1:size(qtraj,1)
    disp(['Calculating joint position trajectory for segment ',num2str(n),'...']);
    qtraj_temp = ikine_at40gw(qtraj(n).at40(:,1:3), q0, usedjoints);
    disp('Done!');

    disp(['Calculating joint velocity trajectory for segment ',num2str(n),'...']);
    qdtraj_temp = cartvel2jointvel_at40gw(qtraj_temp, qtraj(n).at40(:,4:6), usedjoints);
    disp('Done!');
    
    qtraj(n).at40 = [qtraj_temp, qdtraj_temp];
end
disp('AT40GW joint-space toolpath completed!');


%% Convert KUKA points to joint space (add as separate trajectory) and add extra column to en_traj
% NOTE: For now, we're not doing this - we're just putting in dummy values
qtraj_temp = struct2cell(qtraj)'; % Note - struct2cell transposes, so we need to transpose back
qtraj_temp(:,6:7) = qtraj_temp(:,5:6); % Shift over sideways
for n = 1:size(qtraj_temp,1)
    qtraj_temp(n,5) = {zeros(length(qtraj_temp{n,1}),12)}; % This part should be 12 wide, for kuka_q and kuka_qd
end

% Add extra column to en to reflect kuka_q
for n = 1:size(qtraj,1)
    qtraj_temp{n,7}(:,4) = qtraj_temp{n,7}(:,3); % Copy tool to last position
    qtraj_temp{n,7}(:,3) = zeros(size(qtraj_temp{n,7}(:,3)));
end

% Turn back into struct, and rename fields
qtraj = cell2struct(qtraj_temp,{'t' 'dcp' 'at40' 'kuka_x' 'kuka_q' 'tool' 'en'},2);

%% Verify that robots do not exceed task or joint limits
% Eventually, we'll do this for the AT40, KUKA_x and KUKA_q, but now just
% AT40
at40_qtrajtemp = [];
for n = 1:size(qtraj,1)
    at40_qtrajtemp = [at40_qtrajtemp; qtraj(n).at40(:,1:4)];
end
exceeded = checkjointlimits_at40gw(robot, joint2raw_at40gw(robot,at40_qtrajtemp));
if any(exceeded(:))
    disp(['WARNING Joints exceeded limits in trajectory, joints are: ' num2str(find(any(exceeded,1)))]);
end

%% Clear all other variables except for qtraj; close figures
curvars = {curvars{:},'qtraj'};
clearvars('-except', curvars{:});
clear curvars
close all;

%{ 
%% Old crap
qtraj = {carttraj2jointtraj_at40gw(robot,xtraj{1}),xtraj{2}};

qrawtrajs = {j2rfcn_at40gw(qtraj{1}(:,1),qtraj{1}(:,2),qtraj{1}(:,3),qtraj{1}(:,4))};
qdrawtrajs = {jointvel2rawvel_at40gw(robot,qtraj{1}(:,1:4),qtraj{1}(:,5:8))};
%}
