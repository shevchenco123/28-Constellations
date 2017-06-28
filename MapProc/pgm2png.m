cd ./map_store;
ori_map_name = '613';
ori_map_fmt ='.pgm';
im_data = imread([ori_map_name, ori_map_fmt]);
cd ../map_nav;
imwrite(im_data, [ori_map_name,'.png']); 
cd ..;