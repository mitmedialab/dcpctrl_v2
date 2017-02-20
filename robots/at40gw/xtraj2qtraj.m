% XTRAJ2QTRAJ.M Converts task-space (Cartesian) trajectory to joint space

%{

xtraj2qtraj.m
Julian Leland, MIT Media Lab
2017-02-18

This function converts a task-space (Cartesian) trajectory to a
joint-space trajectory. Currently, this is only used for converting raster
toolpaths - also translates into a form that pathviz can use

%}

qtraj = {carttraj2jointtraj_at40gw(robot,xtraj{1}),xtraj{2}};

qrawtrajs = {j2rfcn_at40gw(qtraj{1}(:,1),qtraj{1}(:,2),qtraj{1}(:,3),qtraj{1}(:,4))};
qdrawtrajs = {jointvel2rawvel_at40gw(robot,qtraj{1}(:,1:4),qtraj{1}(:,5:8))};

