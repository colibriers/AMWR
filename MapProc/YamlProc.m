%% Process map YAML file for Modified Map 
%% Create: wwang
%% Time: 20170522

clc;
clear;

yaml_path = 'map_store/fac602.yaml';
fid_ori = fopen(yaml_path);
%% process 1st line for image name
line_1 = fgetl(fid_ori);
len_1 = length(line_1);
name = [line_1(8:end-4), '_mdf'];
new_1 = ['image: ', name, '.pgm'];

%% process 2nd line for resolution
new_resol = 0.05;   % unit: m
line_2 = fgetl(fid_ori);
resol = line_2(13:end);
new_resol_str = num2str(new_resol);
new_2 = ['resolution: ',new_resol_str];

%% process 3rd line for origin
new_ori_x = 1.055; new_ori_y = 1.0; new_ori_yaw = 0.0;
line_3 = fgetl(fid_ori);
new_ori = ['[',num2str(new_ori_x),', ',num2str(new_ori_y), ', ', num2str(new_ori_yaw), ']'];
new_3 = ['origin: ',new_ori];

%% process 4th line for negate
new_negate = 0;
line_4 = fgetl(fid_ori);
new_4 = ['negate: ',num2str(new_negate)];

%% process 5th line for occupy threshold
new_occ_thd = 0.65;
line_5 = fgetl(fid_ori);
new_5 = ['occupied_thresh: ',num2str(new_occ_thd)];

%% process 6th line for occupy threshold
new_free_thd = 0.196;
line_6 = fgetl(fid_ori);
new_6 = ['free_thresh: ',num2str(new_free_thd)];

%% create new yaml
cd map_nav;
mdf_name = [name, '.yaml'];
fid_mdf = fopen(mdf_name, 'w+');

fprintf(fid_mdf, new_1);
fprintf(fid_mdf, '\n');
fprintf(fid_mdf, line_2);
fprintf(fid_mdf, '\n');
fprintf(fid_mdf, line_3);
fprintf(fid_mdf, '\n');
fprintf(fid_mdf, line_4);
fprintf(fid_mdf, '\n');
fprintf(fid_mdf, line_5);
fprintf(fid_mdf, '\n');
fprintf(fid_mdf, line_6);
fprintf(fid_mdf, '\n');

fclose(fid_ori);
fclose(fid_mdf);
