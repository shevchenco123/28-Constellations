function [ ] = MouseSetRectange( src,event )
%MOUSECLICKFCN Summary of this function goes here
%   Detailed explanation goes here
    rec_corner = zeros(2,1);
    pt=get(gca,'CurrentPoint');     %在当前坐标轴中获取鼠标点击的坐标位置
    rec_corner(1, 1) = pt(1,1);
    rec_corner(2, 1) = pt(1,2);
    disp(['Rec Corner(x, y) = ', pt(1,1),', ', pt(1,2)]);
    plot(rec_corner(1, 1), rec_corner(2, 1),'x', 'MarkerSize', 5, 'MarkerEdgeColor', 'y', 'MarkerFaceColor', 'y');
    
end


