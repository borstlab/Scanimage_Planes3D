function tfs = si_isBeamActive(beams)
%SI_ISBEAMACTIVE Returns logical value(s) indicating whether Pockels beam(s) is/are currently active

global state

tfs = arrayfun(@(x)scalarFcn(x),beams);

    function tf = scalarFcn(beam)
        tf = false;

        lists = {'focus' 'grab' 'snap'};
        for i=1:length(lists)
            if strfind(state.init.eom.([lists{i} 'LaserList']),['PockelsCell-' num2str(beam)])
                tf = true;
                break;
            end
        end
    end

end
