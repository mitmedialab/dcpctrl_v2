% NEOPIXEL_UDP.M Sends UDP packets to control NeoPixel bulb for light
% painting.

%{

xtraj2qtraj.m
Julian Leland, MIT Media Lab
2017-08-31

Simple demo showing how to format and send packets to NeoPixel bulb.

%}

packet = ('020,020,020');

u = udp('10.100.48.255',56700); %lightbulb ip address and listening port

%'192.168.0.255'

fopen(u);

fwrite(u, packet, 'uint8');

fclose(u);
delete(u);
clear u;