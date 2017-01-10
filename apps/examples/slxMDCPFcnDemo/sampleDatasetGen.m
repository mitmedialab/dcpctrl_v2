% Sample trajectory dataset - experimenting with bringing workspace
% variables into SLX

dt = 0.01; % Fundamental sampling time of controller
t = linspace(0,100,100/dt);

dist = 2; % Total distance in volts or counts

q = [(dist*sin(0.2*t))]';
q = [q*100000,q,q,q];
qd = [zeros(1,4);diff(q)];

traj = timeseries([q,qd],t);