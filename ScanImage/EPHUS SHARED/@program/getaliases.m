function out = getaliases(obj)
%GETALIASES   - @program method for outputting GUI aliases/mfiles as cell array.
%   GETALIASES(obj) will extract the aliases and mfilenames from the object
%   and output them as a cell array of strings.
%
%   The format is {'Alias1','mfile1','Alias2','mfile2',...}
%
%   See also PROGRAM

out=[];
copy=struct(obj);
aliases=fieldnames(copy.aliases);
number_of_fields=length(aliases);
out=cell(1,2*number_of_fields);
out(1:2:end)=aliases;
for aliasCounter=2:2:2*number_of_fields
    out{aliasCounter}=copy.aliases.(out{aliasCounter-1}).m_filename;
end