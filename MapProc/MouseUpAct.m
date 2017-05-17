function [ ] = MouseUpAct( src,event )
%MOUSECLICKFCN Summary of this function goes here
%   Detailed explanation goes here
    global mouse_down_flag;
    if(mouse_down_flag == 1)   
      mouse_down_flag = 0;
    end
end

