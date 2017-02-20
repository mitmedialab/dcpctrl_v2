function [ qtraj_comp ] = carttraj2jointtraj_at40gw( robot,xtraj_comp )
%CARTTRAJ2JOINTTRAJ_at40gw Convert task-space (Cartesian) trajectory to
%joint-space for the AT40GW

%{
carttraj2jointtraj_at40gw.m
Julian Leland, MIT Media Lab, 2017-02-18

This function takes a time-parametrized, task-space (Cartesian) trajectory
of the form:
    (x(t),y(t),z(t),xd(t),yd(t),zd(t),ts(t))
and converts it to a joint-space trajectory for the AT40GW of the form
    (q1(t),q2(t),q3(t),q4(t),qd1(t),qd2(t),qd3(t),qd4(t),ts(t))

INPUTS:

OUTPUTS:

%}

%% Extract toolpath components
xtraj = xtraj_comp(:,1:3);
xdtraj = xtraj_comp(:,4:6);
ts = xtraj_comp(:,7);

%% Get q0 if not already specified
qraw0 = robot.HomeRawPos;
q0 = r2jfcn_at40gw(qraw0(1),qraw0(2),qraw0(3),qraw0(4));

disp(['Current robot endpoint position;',num2str(q0)]);

usedjoints = logical([1 0 1 1]); % Used joints mask

%% Translate toolpath to current endpoint position
disp('Ofsetting toolpath to current robot endpoint position...');
x0 = joint2cart_at40gw(q0);
offset = x0 - xtraj(1,1:3);
for n = 1:length(xtraj)
    xtraj(n,:) = bsxfun(@plus, xtraj(n,:),offset);
end

%% Convert toolpaths
disp('Calculating joint position trajectory...');
qtraj = ikine_at40gw(xtraj, q0, usedjoints);
disp('Calculating joint velocity trajectory...');
qdtraj = cartvel2jointvel_at40gw(qtraj, xdtraj, usedjoints);

%% Package and return
qtraj_comp = [qtraj,qdtraj,ts];

end

