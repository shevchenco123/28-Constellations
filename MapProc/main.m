%% Modify the Nav Map for Fill and Clear Obs/Grey Area and Set Goal/Init Pos/Nav Point 
%% Create: wwang
%% Time: 20170522

clc;
clear;
close all;
name = 'map_set/fac505';
map_name = [name, '.pgm'];
nav_map = imread(map_name);
disp('Load Map Picture...');

figure(1);
imshow(nav_map);
map_sz = size(nav_map);
len = 50;
h_x = map_sz(1,2)/2.0 -len/2;
h_y = map_sz(1,1)/2.0 -len/2;

h = imrect(gca, [h_x h_y len len]);  %load ROI area
api = iptgetapi(h);  
api.addNewPositionCallback(@(p) title(mat2str(p,3)));  
fcn = makeConstrainToRectFcn('imrect',get(gca,'XLim'),get(gca,'YLim'));  
api.setPositionConstraintFcn(fcn);  

hold on;

[ focus_xy ] = CalcRelativeCoord( [80,298], 0.05, [1,1068])

map_proc = 1;   % branch proc flags for set init/target point,nav path poing,fill/clear/unknown a area
exit_flag = 0;
init_flag = 0;
target_flag = 0;
nav_flag = 0;
fill_flag = 0;
clear_flag = 0;
unknown_flag = 0;

black_gray = 0;     % gray value for modify the ROI and should not changed for ROS systerm
white_gray = 254;
unknown_gray = 205;

while(map_proc)
   op = input('Input map operation mode: ');
   switch op
       %% input 0 exit the map process 
       case 0
           disp('--- Exit Map Process Mode...');
           exit_flag = 1;
           break;
       %% 
       case 1
           disp('+++ Set Robot Target Pos...');
           if(target_flag == 0)
                [rec, center] = CalcRecParam(h);
                plot(center(1, 1), center(1, 2),'p', 'MarkerSize', 8, 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');
                target_flag = 1;
           end
           hold on;
       %%
       case 2
           disp('+++ Set Robot Init Pos...');
           if(init_flag == 0)
                [rec, center] = CalcRecParam(h);
                plot(center(1, 1), center(1, 2),'s', 'MarkerSize', 6, 'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'g');
                init_flag = 1;
           end
           hold on;
       %%
       case 3
           disp('+++ Fill an Rec Area...');
           if(fill_flag == 0)
                [rec, center] = CalcRecParam(h);
                nav_map = ModifyEdgeGreyValue( nav_map, black_gray, rec, 2);
                rectangle('Position',rec,'LineWidth',0.1,'EdgeColor','y');
                plot(center(1, 1), center(1, 2),'+', 'MarkerSize', 4, 'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'y');
                fill_flag = 1;
           end
           hold on;
       %%
       case 4
           disp('+++ Clear an Rec Area...');
           if(clear_flag == 0)
                [rec, center] = CalcRecParam(h);
                nav_map = ModifyGreyValue( nav_map, white_gray, rec);
                rectangle('Position',rec,'LineWidth',0.1,'FaceColor',[0.996 0.996 0.996],'EdgeColor','y');
                plot(center(1, 1), center(1, 2),'.', 'MarkerSize', 6, 'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'y');
                clear_flag = 1;
           end
           hold on;
       %%
       case 5
           disp('+++ Unknow an Rec Area...');
           if(unknown_flag == 0)
                [rec, center] = CalcRecParam(h);
                nav_map = ModifyGreyValue( nav_map, unknown_gray, rec);
                rectangle('Position',rec,'LineWidth',0.1,'FaceColor',[0.804 0.804 0.804],'EdgeColor','y');
                plot(center(1, 1), center(1, 2),'*', 'MarkerSize', 6, 'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'y');
                unknown_flag = 1;
           end
           hold on;
       %%
       case 6
           disp('+++ Set Robot Nav Path Point...');
           if(nav_flag == 0)
                [rec, center] = CalcRecParam(h);
                plot(center(1, 1), center(1, 2),'x', 'MarkerSize', 6, 'MarkerEdgeColor', 'b');
                nav_flag = 1;
           end
           hold on;
       %%
       case 9
           disp('...Reset flags for New Operation...');
           init_flag = 0;
           target_flag = 0;
           nav_flag = 0;
           fill_flag = 0;
           clear_flag = 0;
           unknown_flag = 0;
           
       otherwise
           disp('~~~ An Error Input Mode Param !!!');
   end
   
   if(exit_flag == 1)
       break;
   end
end
delete(h);

%% Read map 1st line for image name
yaml_name = [name, '.yaml'];
fid_ori = fopen(yaml_name, 'r');
line_1 = fgetl(fid_ori);
len_1 = length(line_1);
name = [line_1(8:end-4), '_mdf'];
name_mdf = [name, '.pgm'];

figure(2)
cd map_mdf
imshow(nav_map,'Border','tight');
imwrite(nav_map, name_mdf, 'pgm');

fclose(fid_ori);

