function [image] = fun_read_image(msgStructs)


    msg=msgStructs{1,1};
    a = reshape(msg.Data,3,msg.Width,msg.Height);
    b = permute(a,[3,2,1]);
    image = b(:,:,[3,2,1]);



end

