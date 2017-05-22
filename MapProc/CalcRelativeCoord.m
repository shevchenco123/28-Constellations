function [ focus_xy ] = CalcRelativeCoord( coord_origin, resol, focus_uv)
%CALCRELATIVECOORD Summary of this function goes here
%   Detailed explanation goes here
    focus_xy = (focus_uv - coord_origin) * resol; % except resol, others should be 1x2
    
end

