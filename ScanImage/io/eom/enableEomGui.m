function enableEomGui(on)
global state gh


if ~isfield(gh,'powerControl') %VI011609A
    return;
end

if nargin < 1
    on=1;
end

if ~state.init.eom.pockelsOn %VI112310A
    on=0;
end

panels = {'pnlMainPowerControl'}; %VI010609A
nonpanelkids = findobj(get(gh.powerControl.figure1,'Children'),'flat','-property','Enable'); %VI011210A
if on
    % Commence monkeying.
    %set(get(gh.powerControl.figure1,'Children'),'Enable','on'); %VI010609A
    cellfun(@(x)set(get(gh.powerControl.(x),'Children'),'Enable','on'), panels); %VI010609A
    set(nonpanelkids,'Enable','on'); %VI010609A    
    set(get(gh.powerTransitions.figure1,'Children'),'Enable','on');
       
    %%%VI021009A%%%%
    if get(gh.powerControl.tbShowPowerBox,'Value')
        seeGUI('gh.powerBox.figure1');
    end
    %%%%%%%%%%%%%%%
else
    % Disable monkeying with this stuff.
    %set(get(gh.powerControl.figure1,'Children'),'Enable','off'); %VI010609A
    cellfun(@(x)set(get(gh.powerControl.(x),'Children'),'Enable','off'), panels); %VI010609A
    set(nonpanelkids,'Enable','off'); %VI010609A
    set(get(gh.powerTransitions.figure1,'Children'),'Enable','off');
    hideGUI('gh.powerBox.figure1'); %VI021009A
end

%%%VI011210A%%%%%
if on && state.init.eom.powerVsZActive
    set(get(gh.powerControl.pnlPowerVsZ,'Children'),'Enable','on');
else
    set(get(gh.powerControl.pnlPowerVsZ,'Children'),'Enable','off');
end
%%%%%%%%%%%%%%%%%

