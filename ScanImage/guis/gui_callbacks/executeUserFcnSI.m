function executeUserFcnSI
global state gh
%%%%%%%%%%%%%%%%%%User Function Call%%%%%%%%%%%%%%%%% TPMOD
if state.userFcnGUI.UserFcnOn
    if ~isempty(state.userFcnGUI.UserFcnSelected)
        for fcnCounter=1:length(state.userFcnGUI.UserFcnSelected)
            try
                eval(state.userFcnGUI.UserFcnSelected{fcnCounter}(1:end-2));
                disp([' *** Evaluated UserFcn: ' state.userFcnGUI.UserFcnSelected{fcnCounter}(1:end-2) ...
                    ' at ' datestr(clock) ' ***']);
            catch
                beep;
                disp(['Error in UserFcn: ' state.userFcnGUI.UserFcnSelected{fcnCounter}(1:end-2) '. Skipping']);
                disp(lasterr);
            end
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
