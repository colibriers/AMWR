function [ld, ru] = CalcMirDiagFromRec( rec_pos )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
ld = [round(rec_pos(1,1)), round(rec_pos(1,2)+rec_pos(1,4))];
ru = [round(rec_pos(1,1) + rec_pos(1,3)), round(rec_pos(1,2))];

end

