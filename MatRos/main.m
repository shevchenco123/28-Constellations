%% set external master
% setenv('ROS_MASTER_URI','http://192.168.1.105:11311');
% setenv('ROS_HOSTNAME','AIV-T450');
% setenv('ROS_IP','192.168.0.130');
clc;
clear;
rosinit();
% scandata = rosmessage('colibri_msgs/AngPotnEngy')
laser = rossubscriber('/scan');
apf = rossubscriber('/apf');
rad = -pi/6:pi/360:7*pi/6;
rad_apf = 0:pi/180:pi;
    
for i=1: 1: 1000
%     scandata = receive(laser,1);
%     pscan = polar(rad, (scandata.Ranges)', 'r');
    % [ cartxy ] = Polar2Cart((scandata.Ranges)',[-30,210],0.5);
    % plot(cartxy(1,:),cartxy(2,:),'r');
%     hold on;
    grid on;
    apfdata = receive(apf,1);
    papf = polar(rad_apf, (apfdata.PotentialValue)', 'g');
    % [ apfxy ] = Polar2Cart((apfdata.PotentialValue)',[0,180],1);
    % plot(apfxy(1,:),apfxy(2,:),'g');
    
    drawnow;
    
%     delete(pscan)
%     delete(papf)
    pause(0.02);
end
