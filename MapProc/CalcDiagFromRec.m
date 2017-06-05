function [lu, rd] = CalcDiagFromRec( rec_pos )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
lu = [rec_pos(1,1), rec_pos(1,2)];
rd = [rec_pos(1,1) + rec_pos(1,3), rec_pos(1,2) + rec_pos(1,4)];

end

