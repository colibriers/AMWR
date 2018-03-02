%% set external master
setenv('ROS_MASTER_URI','http://192.168.1.105:11311');
% setenv('ROS_MASTER_URI','http://192.168.0.130:11311');
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

global rf;
rf = rossubscriber('/rf');

global scandata;
global apfdata;
global rfdata;

% figure(1);

scandata = receive(laser,1);
apfdata = receive(apf,1);
rfdata = receive(rf,1);

pscan = polar(rad_scan, (scandata.Ranges)', 'k');

hold on;
papf = polar(rad_apf, (apfdata.PotentialValue)', 'g');
% hold on;
% prf = polar(rad_apf, (rfdata.PotentialValue)', 'r');

t = timer('StartDelay',0 ,'TimerFcn',@TimerCallBack,'Period',0.1,'ExecutionMode','fixedRate');  

start(t);

for i=1: 1: 2000
    delete(pscan);
    delete(papf);
%     delete(prf);
    %drawnow;
    pscan = polar(rad_scan, (scandata.Ranges)', 'k');
    hold on;
    papf = polar(rad_apf, (apfdata.PotentialValue)', 'g');
%     hold on;
%     prf = polar(rad_apf, (rfdata.PotentialValue)', 'r');
    pause(0.02);
end

delete(t);

