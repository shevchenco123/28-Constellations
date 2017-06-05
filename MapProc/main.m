%% Modify the Nav Map for Fill and Clear Obs/Grey Area and Set Goal/Init Pos/Nav Point 
%% Create: wwang
%% Time: 20170522

clc;
clear;
close all;
name = 'map_store/office602';
map_name = [name, '.pgm'];
nav_map = imread(map_name);
disp('Load Map Picture...');

map_proc = 1;
init_flag = 0;
target_flag = 0;
nav_flag = 0;
fill_flag = 0;
clear_flag = 0;
unknown_flag = 0;
exit_flag = 0;

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

black_gray = 0;     % gray value for modify the ROI and should not changed for ROS systerm
white_gray = 254;
unknown_gray = 205;

robot_init = zeros(1, 6);
target_num = 10;
robot_target = zeros(target_num, 6);    % maximum nav point is 10 target
nav_num = 9;
nav_guide_point = zeros(nav_num, 6);  % maximum guide point is 9 during naving to target

target_cnt = 0;
nav_guide_cnt = 0;


%% Read map YAML for param init
yaml_name = [name, '.yaml'];
fid_yaml = fopen(yaml_name, 'r');
line_1 = fgetl(fid_yaml);
len_1 = length(line_1);
name = [line_1(8:end-4), '_mdf'];
name_mdf = [name, '.pgm'];

%% process 2nd line for resolution
line_2 = fgetl(fid_yaml);
resol = str2double(line_2(13:end));

%% process 3rd line for origin
line_3 = fgetl(fid_yaml);
key_str = line_3(10:end-1);
[LD_x, sub_str] = strtok(key_str, ', ');
[LD_y, sub_str] = strtok(sub_str, ', ');
LD_corner = [str2double(LD_x), str2double(LD_y)];


[coord_origin] = CalcMapCoordOrigin( map_sz, resol, LD_corner );

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
                [ target ] = CalcRelativeCoord( coord_origin, resol, center);
                target_cnt = target_cnt + 1;
                if(target_cnt <= target_num )
                    robot_target(target_cnt, :) = [target_cnt, target, 0.0, 5.0, 1];              
                else
                    target_cnt = 1;
                    robot_target(target_cnt, :) = [target_cnt, target, 0.0, 5.0, 1];
                end           
           end
           hold on;
           
           
       %%
       case 2
           disp('+++ Set Robot Init Pos...');
           if(init_flag == 0)
                [rec, center] = CalcRecParam(h);
                plot(center(1, 1), center(1, 2),'s', 'MarkerSize', 6, 'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'g');
                init_flag = 1;              
                [ init ] = CalcRelativeCoord( coord_origin, resol, center);
                robot_init = [0, init, 0.0, 0.0, 0];
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
                [ nav ] = CalcRelativeCoord( coord_origin, resol, center);
                nav_guide_cnt = nav_guide_cnt + 1;
                if(nav_guide_cnt <= nav_num )
                    nav_guide_point(nav_guide_cnt, :) = [target_cnt/10, nav, 0.0, 0.0, 0.0];              
                else
                    target_cnt = 1;
                    nav_guide_point(nav_guide_cnt, :) = [target_cnt/10, nav, 0.0, 0.0, 0.0];
                end         
                
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
           exit_flag = 1;
   end
   
   if(exit_flag == 1)
       break;
   end
end
delete(h);

%% plot the modified map and save into *_mdf.pgm in folder map_nav
figure(2)
cd map_nav
imshow(nav_map,'Border','tight');
imwrite(nav_map, name_mdf, 'pgm');

fclose(fid_yaml);

%% Save the INIT/TARGET/NAV.po
cd ../map_deploy;

init_name = ['init', '.po'];
fid_init = fopen(init_name, 'w+');
fprintf(fid_init, num2str(robot_init));
fclose(fid_init);

target_name = ['target', '.po'];
fid_target = fopen(target_name, 'w+');
index = 1;
while(robot_target(index,1) ~= 0)
    fprintf(fid_target, num2str(robot_target(index, :)));
    fprintf(fid_target, '\n');
    index = index + 1;
end
fclose(fid_target);

nav_name = ['nav', '.po'];
fid_nav = fopen(nav_name, 'w+');
index = 1;
while(nav_guide_point(index,1) ~= 0)
    fprintf(fid_nav, num2str(nav_guide_point(index, :)));
    fprintf(fid_nav, '\n');
    index = index + 1;
end
fclose(fid_nav);
cd ..


