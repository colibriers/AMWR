function [ nav_map ] = ModifyGreyValue( nav_map, gray_set, rec)
%MODIFYGREYVALUE Summary of this function goes here
%   Detailed explanation goes here
    for u = floor(rec(1,1)):1:ceil(rec(1,1)+rec(1,3))
        for v = floor(rec(1,2)):1:ceil(rec(1,2)+rec(1,4))
            nav_map(v, u) = gray_set;
        end
    end
end

