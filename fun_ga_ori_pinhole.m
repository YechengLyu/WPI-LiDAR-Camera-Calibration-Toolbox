function [error] = fun_ga_pinhole(param)
    % read LiDAR and image data samples
    velo = labels(:,3:5);
    ground_truth = labels(:,6:7);
    
    velo = velo';
    ground_truth=ground_truth';

    % use camera model to estimate point position in image coordinate
    [pc_img] = fun_proj_pinhole(param,velo);
    
    % calculate offset of each data sample
    error_rec = zeros(size(velo,2),1);
    for i_error = 1: size(ground_truth,2)
        error_rec(i_error) = norm(pc_img(1:2,i_error)-ground_truth(:,i_error));
    end
    
    % return averave error
    error = mean(error_rec);
end