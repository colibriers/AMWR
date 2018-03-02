function [lu, rd] = CalcDiagFromRec( rec_pos )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
lu = [round(rec_pos(1,1)), round(rec_pos(1,2))];
rd = [round(rec_pos(1,1) + rec_pos(1,3)), round(rec_pos(1,2) + rec_pos(1,4))];

end

