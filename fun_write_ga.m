function [] = fun_write_ga(labels,model)

    % read fun_pc_sel.m file line by line
    C={};
    i_line=0;
    filename = sprintf('fun_ga_ori_%s.m',model);
    fileID = fopen(filename,'r');
    tline = fgetl(fileID);
    while ischar(tline)
        i_line = i_line+1;
        C{i_line,1}=tline;
        tline = fgetl(fileID);
    end
    fclose(fileID);
    
    % insert a line to the file
    C1=cell(size(labels,1),1);
    for i_line=1:size(labels,1)
        C1{i_line,1}=sprintf('%f %f %f %f %f %f %f',labels(i_line,1),labels(i_line,2),labels(i_line,3),labels(i_line,4),labels(i_line,5),labels(i_line,6),labels(i_line,7));
    end
    C=[C(1,1);
        'labels=[';
        C1;
        '];';
        C(2:end,1)];
    
    
    
    % write back to file
    filename = sprintf('fun_ga_%s.m',model);
    fileID = fopen(filename,'w');
    for i_line = 1:length(C)
        fprintf(fileID,C{i_line,1});
        fprintf(fileID,'\n');
    end
    
    fclose(fileID);

    
    
% end

