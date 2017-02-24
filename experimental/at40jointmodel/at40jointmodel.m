% at40jointmodel.m Basic model of AT40GW hydraulic joint

%{
AT40JointModel
Julian Leland, MIT Media Lab
2017-02-08

This script creates a basic model of a generic hydraulic joint on the
AT40GW.
%}

%% Set up workspace
s = tf('s');

%% Create model of hydraulic cylinder
G_cyl = 1/(s+10);

%% Create model of hydraulic valve
G_val = 10000/(s^2 + 100*s + 10000);

%% Create complete plant model (linear)
G_p = G_cyl*G_val