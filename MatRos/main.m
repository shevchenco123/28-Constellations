%% set external master
% setenv('ROS_MASTER_URI','http://192.168.0.100:11311');
% setenv('ROS_HOSTNAME','AIV-T450');
% setenv('ROS_IP','192.168.0.130');
clc;
clear;
close all;

rosshutdown;
delete(timerfind);

rosinit();


rad_scan = -pi/6:pi/360:7*pi/6;
global laser;
laser = rossubscriber('/scan');

rad_apf = 0:pi/180:pi;
global apf;
apf = rossubscriber('/apf');

global scandata;
global apfdata;

% figure(1);

scandata = receive(laser,1);
apfdata = receive(apf,1);
pscan = polar(rad_scan, (scandata.Ranges)', 'r');

hold on;
papf = polar(rad_apf, (apfdata.PotentialValue)', 'g');

t = timer('StartDelay',0 ,'TimerFcn',@TimerCallBack,'Period',0.1,'ExecutionMode','fixedRate');  

start(t);

for i=1: 1: 6000
    delete(pscan);
    delete(papf);
    %drawnow;
    pscan = polar(rad_scan, (scandata.Ranges)', 'r');
    hold on;
    papf = polar(rad_apf, (apfdata.PotentialValue)', 'g');
    pause(0.02);
end

delete(t);

