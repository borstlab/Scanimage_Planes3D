function tf = si_isPureContinuous
%SI_ISPURECONTINUOUS Macro' function determining when acquisition is 'pure' continuous (i.e. continuous AO and AI) for upcoming acquisition

global state

tf = state.internal.gapFreeAdvanceNext && state.internal.looping; %VI111110A: Pseudo-focus mode has been eliminated %VI092210A: now always save during acquisition

