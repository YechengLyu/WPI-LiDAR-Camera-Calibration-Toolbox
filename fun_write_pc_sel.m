function [] = fun_write_pc_sel(str)
%FUN_WRITE_PC_SEL Summary of this function goes here
%   Detailed explanation goes here
%     str = 'x>2';
    
    % read fun_pc_sel.m file line by line
    C={};
    i_line=0;
    fileID = fopen('fun_pc_sel.m','r');
    tline = fgetl(fileID);
    while ischar(tline)
        i_line = i_line+1;
        C{i_line,1}=tline;
        tline = fgetl(fileID);
    end
    fclose(fileID);
    
    % insert a line to the file
    C{i_line+1,1}=C{i_line,1};
    C{i_line,1}=sprintf('idx = idx .* (%s);',str);
    
    
    
    % write back to file
    fileID = fopen('fun_pc_sel.m','w');
    for i_line = 1:length(C)
        fprintf(fileID,C{i_line,1});
        fprintf(fileID,'\n');
    end
    
    fclose(fileID);

    
    % visulize
    disp('Current filter(s):')
    for i_line = 7:length(C)-1
        line = sscanf(C{i_line,1},'idx = idx .* (%s);');
        fprintf(line(1:end-2));
        fprintf('\n');   
    end
    
end

