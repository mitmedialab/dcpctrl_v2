%{

kukaudpsim.m
Julian Leland
MIT Media Lab, 2016-09-15

This script simulates the behavior of a KUKA KRC4 controller when
communicating via UDP through the RSI interface. It can be used to test the
behavior of the kukaslxctrl model. 

To use, ensure that the Simulink Stream Input/Stream Output blocks are
configured as follows:
- Stream Input:
    - Data acquisition: UDP Protocol, local port 12002, remote port 12000,
    IP 127.0.0.1
    - Sample time: 0.1
- Stream Output:
    - Data acquisition: UDP Protocol, local port 12001, remote port 12000,
    IP 127.0.0.1
    - Sample time: 0.1

NOTE: This has been observed to crash computers, particularly if
kukaslxctrl is running before the UDP communications are set up. Make sure
to save and commit your work before running!

%}

% Set up UDP comms to read from local
u_In = udp('127.0.0.1',12001)
u_In.LocalPort = 12000;
u_Out = udp('127.0.0.1',12002)
u_Out.LocalPort = 12000;

% Define parts of out_packet. MATLAB is pretending to be the KUKA here.
out_packetStart = '<Rob Type="KUKA"><RIst X="540.4" Y="0.0" Z="915.0" A="180.0" B="0.0" C="180.0"/><AIPos A1="0.0" A2="-90.0" A3="90.0" A4="0.0" A5="90.0" A6="0.0"/><Delay D="0"/><IPOC>';
out_packetEnd = '</IPOC></Rob>';
IPOC = 1343092; % Random IPOC value - test to make sure that correct value is sent.

%% Run to read a single packet and send a single packet back to KUKA
fopen(u_Out);
out_packet = [out_packetStart,num2str(IPOC),out_packetEnd];
fprintf(u_Out,out_packet);
fclose(u_Out);

fopen(u_In);
IPOC = IPOC + 1; % Increment IPOC by 1
in_packet = fscanf(u_In)
fclose(u_In);

%% Cleanup
fclose([u_Out u_In])
delete([u_Out u_In])
clear u_Out u_In
