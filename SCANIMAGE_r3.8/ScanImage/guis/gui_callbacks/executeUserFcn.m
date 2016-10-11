function executeUserFcn
global state gh

%%%%%%%%%%%%%%%%%%User Function Call%%%%%%%%%%%%%%%%% TPMOD
if state.userFcnGUI.UserFcnOn
    if  ~isempty(state.userFcnGUI.UserFcnSelected)
        selFunCell = si_transformStringListType(state.userFcnGUI.UserFcnSelected); %VI120109C
        for fcnCounter=1:length(selFunCell) %VI120109C
            try
                %eval(state.userFcnGUI.UserFcnSelected{fcnCounter}(1:end-2)); %VI120109A
                feval(selFunCell{fcnCounter}(1:end-2)); %VI120109C %VI120109A: Use feval()
                disp([' *** Evaluated UserFcn: ' selFunCell{fcnCounter} ... %VI120109C
                    ' at ' datestr(clock) ' ***']);
            catch
                beep;
                disp(['Error in UserFcn: ' selFunCell{fcnCounter} '. Skipping']); %VI120109C
                disp(lasterr);
            end
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
