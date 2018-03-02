function [ focus_xy ] = CalcRelativeCoord( coord_origin, resol, focus_uv)
%CALCRELATIVECOORD Summary of this function goes here
%   Detailed explanation goes here
    focus_xy(1, 1) = (focus_uv(1, 1) - coord_origin(1,1)) * resol; % except resol, others should be 1x2
    focus_xy(1, 2) = (coord_origin(1,2) - focus_uv(1, 2)) * resol;
end

