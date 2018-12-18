function [pc] = fun_read_pc(msgStructs)

    msg = msgStructs{1,1};
    msg_width = msg.Width;
    pc_msg = msg.Data;
    pc = zeros(5,msg_width);
    for i_pc = 1:msg_width
        str = pc_msg(i_pc*32-31:i_pc*32);
        x = typecast( str(1:4) , 'single');
        y = typecast( str(5:8) , 'single');
        z = typecast( str(9:12) , 'single');
        i = typecast( str(17:20) , 'single');
        r = typecast( str(21:22) , 'uint16');
        pc(:,i_pc)=[x,y,z,i,single(r)]';
    end


end

