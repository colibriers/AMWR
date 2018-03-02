cd ./map_nav;
ori_map_name = '626_mdf';
ori_map_fmt ='.pgm';
im_data = imread([ori_map_name, ori_map_fmt]);
cd ../map_nav;
imwrite(im_data, [ori_map_name,'.png']); 
cd ..;