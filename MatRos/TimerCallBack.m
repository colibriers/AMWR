function [ ] = TimerCallBack(hObject,eventdata) 
%TIMERCALLBACK Summary of this function goes here
%   Detailed explanation goes here
    global scandata;
    global laser;
    global apfdata;
    global apf;
    global rf;
    global rfdata;

    scandata = receive(laser,1);
    apfdata = receive(apf,1); 
%     rfdata = receive(rf,1);
end