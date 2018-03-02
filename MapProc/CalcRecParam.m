function [ rec_pos, center] = CalcRecParam( h )
%CALCRECPARAM Summary of this function goes here
%   Detailed explanation goes here
    rec_pos = getPosition(h);
    disp(['LUCorner(x,y) = ',num2str(rec_pos(1,1)), ',', num2str(rec_pos(1,2)),',Rec(W,H):',num2str(rec_pos(1,3)), ',', num2str(rec_pos(1,4))]);
    center = [rec_pos(1,1)+rec_pos(1,3)/2.0, rec_pos(1,2)+rec_pos(1,4)/2.0];
    disp(['Center(u,v) = ',num2str(center(1,1)), ',', num2str(center(1,2))]);
end

