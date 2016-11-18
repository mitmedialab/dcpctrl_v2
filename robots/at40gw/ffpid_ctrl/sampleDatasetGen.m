% Sample trajectory dataset - experimenting with bringing workspace
% variables into SLX

dt = 0.01; % Fundamental sampling time of controller
t = linspace(0,100,10/dt);

q = [(5*sin(0.1*t)+5)]';
qd = [0;diff(q)];

traj.Time = t;
traj.signals(1).values = q;
traj.signals(2).values = qd;

traj = timeseries([q,qd],t);