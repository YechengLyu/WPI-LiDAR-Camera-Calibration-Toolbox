function fun_export_calibration(R,T,theta_d,K)

    if(isempty(theta_d))
        model = 'pinhole';
    else 
        model = 'fisheye';
    end
    fileID = fopen(sprintf('calib_LiDAR_to_cam_%s.txt',model),'w');
    
    time = clock;

    fprintf(fileID, 'calib_time: %d.%d.%d %d:%d\n', time(1),time(2),time(3),time(4),time(5) );
    

    fprintf(fileID, 'Use %s camera model.\n',model);


    fprintf(fileID, 'All parameters are stored in row vectors.\n');   
    fprintf(fileID, 'Param name (rows x cols): values.\n');
    fprintf(fileID, 'R(3x3): %f %f %f %f %f %f %f %f %f\n',R(1),R(2),R(3),R(4),R(5),R(6),R(7),R(8),R(9));
    fprintf(fileID, 'T(3x1): %f %f %f\n',T(1),T(2),T(3));
    if(strcmp( model, 'fisheye' ))
        fprintf(fileID, 'theta_d(1x5): 1 %f %f %f %f\n',theta_d(1),theta_d(2),theta_d(3),theta_d(4));
    end
    fprintf(fileID, 'K(3x3): %f %f %f %f %f %f %f %f %f\n',K(1),K(2),K(3),K(4),K(5),K(6),K(7),K(8),K(9));
    


    fclose(fileID);



end

