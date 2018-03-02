function [ nav_map ] = ModifyEdgeGreyValue(nav_map, gray_set, rec, thick)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    for u = floor(rec(1,1)):1:ceil(rec(1,1)+thick)
        for v = floor(rec(1,2)):1:ceil(rec(1,2)+rec(1,4))
            nav_map(v, u) = gray_set;
        end
    end

    for u = floor(ceil(rec(1,1)+rec(1,3))-thick):1:ceil(rec(1,1)+rec(1,3))
        for v = floor(rec(1,2)):1:ceil(rec(1,2)+rec(1,4))
            nav_map(v, u) = gray_set;
        end
    end
    
    for v = floor(rec(1,2)):1:ceil(rec(1,2)+thick)
        for u = floor(rec(1,1)):1:ceil(rec(1,1)+rec(1,3))
            nav_map(v, u) = gray_set;
        end
    end

    for v = floor(ceil(rec(1,2)+rec(1,4))-thick):1:ceil(rec(1,2)+rec(1,4))
        for u = floor(rec(1,1)):1:ceil(rec(1,1)+rec(1,3))
            nav_map(v, u) = gray_set;
        end
    end
    
    
end

