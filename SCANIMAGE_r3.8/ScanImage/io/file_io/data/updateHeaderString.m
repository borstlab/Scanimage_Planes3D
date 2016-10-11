function updateHeaderString(globalName)
%%  Changed:
%%           Tim O'Connor 2/24/04 TO22404a - Allow multidimensional arrays to be stored.
%%           Tim O'Connor 5/18/04 TO051804c - Save ordinary numbers/arrays, not via ndArray2Str, to
%%                                            coincide with Alex's analysis software.
	global state
try
	flag=getGlobalConfigStatus(globalName);
	if ~bitand(flag, 2)
		return
	end
	pos=findstr(state.headerString, [globalName '=']);
	
	val=eval(globalName);
    
    if islogical(val)
        if val
            val = 1;
        else
            val = 0;
        end
    end
    
	if ~isnumeric(val) & ~ischar(val)
		disp(['updateHeaderString: unknown type for ' globalName]);
		val='0';
	elseif length(size(val)) > 2 & isnumeric(val)
          %TO22404a - Allow multidimensional values. -- Tim O'Connor.
          val = strcat('''', ndArray2Str(val), '''');
    elseif isnumeric(val)
          %Save these as ordinary numbers/arrays, not via ndArray2Str, to coincide with
          %Alex's analysis software. - Tim O'Connor TO051804c 5/18/04
          val = mat2str(val);
	else
		val=['''' val ''''];
	end

	if length(pos)==0
		state.headerString=[state.headerString globalName '=' val 13];
	else
		cr=findstr(state.headerString, 13);
		index=find(cr>pos);
		next=cr(index(1));
		if length(next)==0
			state.headerString=[state.headerString(1:pos-1) globalName '=' val 13];
		else
			state.headerString=[state.headerString(1:pos-1) globalName '=' val state.headerString(next:end)];
		end
	end
end