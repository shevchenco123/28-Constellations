clc;
clear;
close all;
map_proc = 1;
exit_flag = 0;

global nav_path;
global point_cnt;
nav_path = zeros(2,10);
point_cnt = 0;

map_name = 'map_set/fac505.pgm';
nav_map = imread(map_name);
disp('Load Map Picture...')
imshow(nav_map);
hold on;

global mouse_down_flag
mouse_down_flag = 1;
global target
target = zeros(2,1);


while(map_proc)
   op = input('Input map operation mode: ');
   switch op
       case 0
           disp('Exit Map Process Mode...');
           exit_flag = 1;
           break;
       case 1
           disp('Set Robot Target Pos...');
           set(gcf,'WindowButtonDownFcn',@MouseDownAct);
           set(gcf,'WindowButtonUpFcn',@MouseUpAct);
           hold on;
       case 2
           disp('Selet a Rectangle Area...');
           set(gcf,'WindowButtonDownFcn',@MouseSetRectange);
           hold on;         
       case 9
           disp('Waiting for New Operation...');
           mouse_down_flag = 1;
       otherwise
           disp('An Error Input Number !!!');
   end
   
   if(exit_flag == 1)
       break;
   end
end

