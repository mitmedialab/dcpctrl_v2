% Sample trajectory dataset - experimenting with bringing workspace
% variables into SLX

dt = 0.01; % Fundamental sampling time of controller
t = linspace(0,100,10/dt);

dist = 100; % Total distance in millimeters that the path should traverse

q = [(dist*sin(0.1*t))]';
qd = [0;diff(q)];

traj = timeseries([q,qd],t);