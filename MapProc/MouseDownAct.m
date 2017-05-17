function [ ] = MouseDownAct( src,event )
%MOUSECLICKFCN Summary of this function goes here
%   Detailed explanation goes here
    global mouse_down_flag;
    global target;
    if(mouse_down_flag == 1)   
        pt=get(gca,'CurrentPoint');     %在当前坐标轴中获取鼠标点击的坐标位置
        target(1,1) = pt(1,1);
        target(2,1) = pt(1,2);
        disp(['Target(x, y) = ',num2str(target(1,1)), ', ', num2str(target(2,1))]);
        plot(target(1,1), target(2,1),'p', 'MarkerSize', 10, 'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'g');
    end
end

