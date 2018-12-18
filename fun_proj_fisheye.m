function [pc_img] = fun_proj_fisheye(param,velo)
    
    % Extrinsic matrix
    eul = [param(1),param(2),param(3)];
    R_ex = eul2rotm(eul,'ZYX'); % lidar to camera, aka. pose of lidar in camera coordinate
    T_ex = [param(4) param(5) param(6)]'; % lidar to camera, aka. pose of lidar in camera coordinate
    Tr_ex = [R_ex, T_ex];
    
    % Intrinsic matrix
    focus = [param(7),param(8)];
    offset = [param(9),param(10)];
    K_in = [focus(1), 0, offset(1);
        0, focus(2), offset(2);
        0, 0, 1];

    % LiDAR to camera coordinate    
    pc_velo = ones(4,size(velo,2));
    pc_velo(1:3,:)=velo(1:3,:);
    pc_cam = Tr_ex*pc_velo;
    
    % fisheye model
    r =vecnorm(pc_cam(1:2,:).^2,2);
    theta = atan2(r,abs(pc_cam(3,:)));
    theta_d =theta.*(1+param(11)*theta.^2+param(12)*theta.^4+param(13)*theta.^6+param(14)*theta.^8);
    pc_fisheye(1,:) = theta_d.*pc_cam(1,:)./r;
    pc_fisheye(2,:) = theta_d.*pc_cam(2,:)./r;
    pc_fisheye(3,:) = 1;
    
    % camera to image
    pc_img = K_in*pc_fisheye;
    

end

