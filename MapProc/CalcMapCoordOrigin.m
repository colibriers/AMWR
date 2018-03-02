function [coord_origin] = CalcMapCoordOrigin( map_size,resol, LD_corner )
%CALCFOCUSXY Summary of this function goes here
%   Detailed explanation goes here
    coord_origin(1,1) = abs(round(LD_corner(1,1)/resol)); %coord_origin should be int
    coord_origin(1,2) = map_size(1,1) + round(LD_corner(1,2)/resol);
end

