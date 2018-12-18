close all;clc;%clear
%% ReadMe
% This LiDAR-camera toolbox helps users calibrate the transformatin from
% LiDAR cooridnate to camera coordinate. This toolbox take advantage of
% generic algorithm to calculate the parameters in the transforamtion.
% 
% This toolbox is very easy to use: 
% (1) Prepare a piece of board bigger than your head.
% (2) Hold the the board with one corner up.
% (3) Record the LiDAR and camera frames using rosbag as you move around
%     with the board. 
%     * Please not show your head during the recording.
%     * Camera frame rate is recommended to be 1.5x LiDAR frame rate or
%       more for best LiDAR-camera frame matching.
% (4) Use the toolbox to load the rosbag
% (5) Set up threshold for LiDAR detections.
%     Ex. x>0, y<2, y>-2, Z<1, r<5
% (6) Choose to use pinhole camera model or fisheye camera model
% (7) Type in the position of board upper corner in the image associate to
%     selected LiDAR frames.
% (8) The toolbox will use generic algorithm to estimate the
%     transformation parameters.


%% Load rosbag


list = ls('*.bag');
if(size(list,1))
    latest_bag_path = list(end,:);
else
    latest_bag_path=[];
end
bag_path = input(sprintf('>>Please enter the bag path: [%s]\n>>',latest_bag_path),'s');
if isempty(bag_path)
  bag_path = latest_bag_path;
end

while(~exist(bag_path,'file'))
    bag_path = input('Cannot find the rosbag, please enter the bag path again:\n>>','s');
end
bag = rosbag(bag_path);
fprintf('Rosbag loaded: [%s]\n',bag_path)

% type in LiDAR rostopic
LiDAR_topic =  '/velodyne_points';
str = input(sprintf('Please enter the LiDAR topic:[%s]\n>>',LiDAR_topic),'s');
if(~isempty(str))
    LiDAR_topic = str;
end

% type in camera rostopic
image_topic = '/camera/image_color';
str = input(sprintf('Please enter the image topic:[%s]\n>>',image_topic),'s');
if(~isempty(str))
    image_topic = str;
end

bSel_LiDAR = select(bag,'Topic','/velodyne_points');
bSel_image = select(bag,'Topic','/camera/image_color');

LiDAR_frame_count = size(bSel_LiDAR.MessageList,1);
image_frame_count = size(bSel_image.MessageList,1);

fprintf('LiDAR data loaded: [%d] frames.\n',LiDAR_frame_count);
fprintf('Image data loaded: [%d] frames.\n',image_frame_count);


%% Set threshold


frame_id = round(LiDAR_frame_count/2);
msgStructs = readMessages(bSel_LiDAR,frame_id);
pc = fun_read_pc(msgStructs);

plot3(pc(1,:),pc(2,:),pc(3,:),'.b');
xlabel('x')
ylabel('y')
zlabel('z')
axis('equal')

copyfile fun_pc_sel_ori.m fun_pc_sel.m;

flag_sel = 0;
% start threshold
while(flag_sel < 1)
    str = input('>>Please set the threshold:\n>>','s');
    
    if(~isempty(str))
        fun_write_pc_sel(str);
        try
            idx = fun_pc_sel(pc);       
        catch ME
            disp('>>Input expression is not valid. Drop it and input again.')
            fun_drop_pc_sel();
            continue;
        end
    end
    
%     idx = fun_pc_sel(pc);
    pc_sel = pc(:,logical(idx));

    % visulization
    r = vecnorm(pc(1:3,:));
    idx_present = (1-idx);
    pc_present = pc(:,logical(idx_present));


    plot3(pc_present(1,:),pc_present(2,:),pc_present(3,:),'.b');
    hold on
    plot3(pc_sel(1,:),pc_sel(2,:),pc_sel(3,:),'.r');
    hold off
    xlabel('x')
    ylabel('y')
    zlabel('z')
    axis('equal')
    
    str = input('Is the threshold filter good now? [(y)es/(n)o/(d)rop]:','s');
    if(length(str)~=1)
        str='N';
        disp('Invalid input, turn to default choice: no.')
    end

    
    if(str=='y' || str=='Y')
        flag_sel = 1;
    end
    
    if(str=='d' || str=='D')
        fun_drop_pc_sel();
    end      
end

%% Label data

dir_name = 'selected_frames';
mkdir(dir_name);
close all;
labels=zeros(0,7); % [LiDAR_frame_id, image_frame_id,x,y,z,u,v]

for LiDAR_frame_id = 74:LiDAR_frame_count
    
    % read LiDAR point cloud
    msgStructs = readMessages(bSel_LiDAR,LiDAR_frame_id);
    
    pc = fun_read_pc(msgStructs);
    
    idx = fun_pc_sel(pc);
    pc_sel = pc(:,logical(idx));
    
    % if too few points are selected, drop the frame
    if(size(pc_sel,2)<10)
        continue;
    end
    
    % if too many points on the top ring are found (not a corner),
    % drop the frame
    ring_max = max(pc_sel(5,:));
    [~,idx_conor] = find(pc_sel(5,:)==ring_max);
    if(length(idx_conor)>3)
        continue;
    end
    
    % if points on the top ring are far away (not a corner),
    % drop the frame   
    pc_corner = pc_sel(1:3,idx_conor);
    distance_matrix = pdist2(pc_corner',pc_corner','euclidean');
    if(max(distance_matrix(:))>0.05)
        continue;
    end
    
    % label the position of detected corner in LiDAR coordinate
    x_LiDAR = mean(pc_corner(1,:));
    y_LiDAR = mean(pc_corner(2,:));
    z_LiDAR = mean(pc_corner(3,:));
    
    
    
    % present a larger view for validation
    r = vecnorm(pc(1:3,:));
    idx_present = (r<10).*(1-idx);
    pc_present = pc(:,logical(idx_present));
    
    
    % get associate camera frame
    LiDAR_time = bSel_LiDAR.MessageList.Time(LiDAR_frame_id);
    [~, image_frame_id]=min(abs(bSel_image.MessageList.Time-LiDAR_time));
    
    msgStructs = readMessages(bSel_image,image_frame_id);
    image = fun_read_image(msgStructs);
    
    
    % visualization
    figure(1);
    hold on
    plot3(pc_present(1,:),pc_present(2,:),pc_present(3,:),'.b');
    plot3(pc_sel(1,:),pc_sel(2,:),pc_sel(3,:),'.r');
    plot3(x_LiDAR,y_LiDAR,z_LiDAR,'.g','MarkerSize',20);
    
    hold off
    xlabel('x')
    ylabel('y')
    zlabel('z')
    axis('equal')  
    
    fig=figure(2);
    imshow(image)
    
    
    dcm_obj = datacursormode(fig);
    
    set(dcm_obj,'DisplayStyle','datatip',...
    'SnapToDataVertex','off','Enable','on')
    
    flag_uv = 0;
    while(flag_uv == 0)
        str = input('>>Please point the corner in the image and press Enter or N to drop the frame:\n>>','s');
        c_info = getCursorInfo(dcm_obj);
        if(isempty(str))
            if(isempty(c_info))
                disp('>>Invalid input.');
                continue;
            else
               labels = [labels; LiDAR_frame_id, image_frame_id, x_LiDAR,y_LiDAR,z_LiDAR,c_info.Position];
              
               % extract selected frames
               imwrite(image,sprintf('%s/image_%06d.png',dir_name,image_frame_id));
               fileID = fopen(sprintf('%s/LiDAR_%06d.txt',dir_name,LiDAR_frame_id),'w');
               fprintf(fileID,'%f %f %f %f %f\n',pc);
               fclose(fileID);
            end
        end

        if(str == 'N' || str=='n')
            flag_uv = 1;
        end
    end
    
    
    
    
end
save('labels','labels')

% write labels to file;
fileID = fopen('labels.txt','w');
for i_line = 1:size(labels,1)
    fprintf(fileID,'%f %f %f %f %f %f %f\n',labels(i_line,1),labels(i_line,2),labels(i_line,3),labels(i_line,4),labels(i_line,5),labels(i_line,6),labels(i_line,7));
end
fclose(fileID);

%% Setting calibration settings

% select the camera model to be used.
flag_pinhole = -1;
flag_fisheye = -1;
while (flag_pinhole+flag_fisheye == -2)
    str = input('>>Do you want to try pinhole camera model? [Y/N]\n>>','s');
    if(str=='Y')
        flag_pinhole = 1;
    end
    if(str=='N')
        flag_pinhole = 0;
    end  

    str = input('>>Do you want to try fisheye camera model? [Y/N]\n>>','s');
    if(str=='Y')
        flag_fisheye = 1;
    end
    if(str=='N')
        flag_fisheye = 0;
    end  
    
    if(flag_pinhole+flag_fisheye == -2)
        disp('Invalid input. At lease one model should be selected.')
    end
    
end


%% Calibration using pinhole model
if(flag_pinhole == 1)
    
    
    % prepare ga funtion file
    fun_write_ga(labels,'pinhole');    
    
    flag_lb=0;
    flag_ub=0;
    
    %default boundaries.
%     lb_pinhole = [pi/2-0.3*pi, -pi/2-0.3*pi, -0.3*pi, -1, -1, -1,  300,  300, 300, 300];
%     ub_pinhole = [pi/2+0.3*pi, -pi/2+0.3*pi,  0.3*pi,  1,  1,  1, 1000, 1000, 900, 900];
    lb_pinhole = [0.628318530717959,-2.51327412287183,-0.942477796076938,-1,-1,-1,300,300,300,300];
    ub_pinhole = [2.51327412287183,-0.628318530717959,0.942477796076938,1,1,1,1000,1000,900,900];

    while (flag_lb<1)
        str = input('>>Please set the lower boundary:\n>>yaw,pitch,roll,tx,ty,tz,focus_x,focus_y,offset_x,offset_y\n>>','s');
        if(~isempty(str))
            lb_pinhole_input = sscanf(str,'%f,%f,%f,%f,%f,%f,%f,%f,%f,%f');
        end
        if(length(lb_pinhole_input)==10)
            flag_lb=1;
            lb_pinhole=lb_pinhole_input;
        end
    end
    while (flag_ub<1)
        str = input('>>Please set the upper boundary:\n>>yaw,pitch,roll,tx,ty,tz,focus_x,focus_y,offset_x,offset_y\n>>','s');
        if(~isempty(str))
            ub_pinhole_input = sscanf(str,'%f,%f,%f,%f,%f,%f,%f,%f,%f,%f');
        end
        if(length(ub_pinhole_input)==10)
            flag_ub=1;
            ub_pinhole = ub_pinhole_input;
        end
    end


    error_pinhole_rec = ones(20,1)*1e12;
    param_pinhole_rec = zeros(20,10);

    for i_ga =1:20
        options = optimoptions('ga','MaxGenerations',50,'PopulationSize',2000,...
            'MaxStallGenerations',Inf,'PlotFcn',@gaplotbestf);
        param_pinhole = ga(@fun_ga_pinhole,10,[],[],[],[],lb_fisheye,ub_fisheye,[],options);
        error = fun_ga_pinhole(param_pinhole);
        error_pinhole_rec(i_ga,1)=error;
        param_pinhole_rec(i_ga,:)=param_pinhole;
    end

    [error,i_min]=min(error_pinhole_rec);
    param_pinhole = param_pinhole_rec(i_min,:);
    fprintf('>>Calibration using pinhole model finished, the average error is %f pixel\n', error);

    save('param_pinhole','param_pinhole')
end

%% export pinhole result
% Extrinsic matrix
eul = [param(1),param(2),param(3)];
R = eul2rotm(eul,'ZYX'); % lidar to camera, aka. pose of lidar in camera coordinate
T = [param(4) param(5) param(6)]'; % lidar to camera, aka. pose of lidar in camera coordinate
Tr_pinhole = [R, T];

% Intrinsic matrix
focus = [param(7),param(8)];
offset = [param(9),param(10)];

K = [focus(1),0,offset(1);
    0,focus(2),offset(2);
    0,      0,         1];

fun_export_calibration(R,T,[],K);

%% visulize example result: pinhole

LiDAR_frame_id = round(LiDAR_frame_count/2);
msgStructs = readMessages(bSel_LiDAR,LiDAR_frame_id);
pc = fun_read_pc(msgStructs);
idx = fun_pc_sel(pc);
pc_sel = pc(:,logical(idx));

    
% get associate camera frame
LiDAR_time = bSel_LiDAR.MessageList.Time(LiDAR_frame_id);
[~, image_frame_id]=min(abs(bSel_image.MessageList.Time-LiDAR_time));

msgStructs = readMessages(bSel_image,image_frame_id);
image = fun_read_image(msgStructs);

% project LiDAR point cloud to image
pc_img = fun_proj_pinhole(param_pinhole,pc_sel(1:3,:));

    
figure;
imshow(image);
hold on
plot(pc_img(1,:),pc_img(2,:),'.b');
hold off
title('visulize example result: pinhole model')


%% Calibration using fisheye model


if(flag_fisheye == 1)
    
    % prepare ga funtion file
    fun_write_ga(labels,'fisheye');
    
    flag_lb=0;
    flag_ub=0;
    
    %default boundaries.
    lb_fisheye = [pi/2-0.3*pi, -pi/2-0.3*pi, -0.3*pi, -1, -1, -1,  300,  300, 300, 300, -2e-1, -2e-1, -2e-1, -2e-1];
    ub_fisheye = [pi/2+0.3*pi, -pi/2+0.3*pi,  0.3*pi,  1,  1,  1, 1000, 1000, 900, 900,  2e-1,  2e-1,  2e-1,  2e-1];


    while (flag_lb<1)
        str = input('>>Please set the lower boundary:\n>>yaw,pitch,roll,tx,ty,tz,focus_x,focus_y,offset_x,offset_y,theta3,theta5,theta7,theta9\n>>','s');
        if(~isempty(str))
            lb_fisheye_input = sscanf(str,'%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f');
        end
        if(length(lb_fisheye_input)==14)
            flag_lb=1;
            lb_fisheye=lb_fisheye_input;
        end
    end
    while (flag_ub<1)
        str = input('>>Please set the upper boundary:\n>>yaw,pitch,roll,tx,ty,tz,focus_x,focus_y,offset_x,offset_y,theta3,theta5,theta7,theta9\n>>','s');
        if(~isempty(str))
            ub_fisheye_input = sscanf(str,'%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f');
        end
        if(length(ub_fisheye_input)==14)
            flag_ub=1;
            ub_fisheye=ub_fisheye_input;
        end
    end


    error_fisheye_rec = ones(20,1)*1e12;
    param_fisheye_rec = zeros(20,14);

    for i_ga =1:20
        options = optimoptions('ga','MaxGenerations',50,'PopulationSize',2000,...
            'MaxStallGenerations',Inf,'PlotFcn',@gaplotbestf);
        param_fisheye = ga(@fun_ga_fisheye,14,[],[],[],[],lb_fisheye,ub_fisheye,[],options);
        error = fun_ga_fisheye(param_fisheye);
        error_fisheye_rec(i_ga,1)=error;
        param_fisheye_rec(i_ga,:)=param_fisheye;
    end

    [error,i_min]=min(error_fisheye_rec);
    param_fisheye = param_fisheye_rec(i_min,:);
    fprintf('>>Calibration using fisheye model finished, the average error is %f pixel\n', error);

    save('param_fisheye','param_fisheye')
end
%% export result
% Extrinsic matrix
eul = [param(1),param(2),param(3)];
R = eul2rotm(eul,'ZYX'); % lidar to camera, aka. pose of lidar in camera coordinate
T = [param(4) param(5) param(6)]'; % lidar to camera, aka. pose of lidar in camera coordinate
Tr = [R, T];


% Fisheye camera model
theta_d = [1, param(11), param(12), param(13), param(14)];

% Intrinsic matrix
focus = [param(7),param(8)];
offset = [param(9),param(10)];

K = [focus(1),0,offset(1);
    0,focus(2),offset(2);
    0,      0,         1];

fun_export_calibration(R,T,theta_d,K);

%% visulize example result: fisheye model

LiDAR_frame_id = round(LiDAR_frame_count/2);
msgStructs = readMessages(bSel_LiDAR,LiDAR_frame_id);
pc = fun_read_pc(msgStructs);
idx = fun_pc_sel(pc);
pc_sel = pc(:,logical(idx));

    
% get associate camera frame
LiDAR_time = bSel_LiDAR.MessageList.Time(LiDAR_frame_id);
[~, image_frame_id]=min(abs(bSel_image.MessageList.Time-LiDAR_time));

msgStructs = readMessages(bSel_image,image_frame_id);
image = fun_read_image(msgStructs);

% project LiDAR point cloud to image
pc_img = fun_proj_fisheye(param_fisheye,pc_sel(1:3,:));

    
figure;
imshow(image);
hold on
plot(pc_img(1,:),pc_img(2,:),'.b');
hold off
title('visulize example result: fisheye model')


