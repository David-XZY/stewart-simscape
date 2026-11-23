function ch5LaunchProbe(stage,selection)
% Exercise the actual Windows -batch argument path without drawing holdouts.
persistent seen
if isempty(seen),seen={};end
expected={'development',1,'ideal-force',0; ...
          'development',12,'nonideal-force',5101; ...
          'formal',120,'nonideal-force',5105};
k=numel(seen)+1;assert(k<=size(expected,1));
assert(strcmp(stage,expected{k,1}));
assert(selection.indices==expected{k,2});
assert(isscalar(string(selection.modes))&&strcmp(selection.modes,expected{k,3}));
assert(selection.seeds==expected{k,4});
assert(strcmp(selection.worker,'campaign_1'));
seen{end+1}=struct('stage',stage,'selection',selection); %#ok<AGROW>
fprintf('Actual argv parsed: %s, %s, index %d, seed %d\n',stage,selection.modes,selection.indices,selection.seeds);
if k==size(expected,1)
    report=struct('passed',true,'cases',{seen},'holdout_generated',false);
    save(fullfile(fileparts(mfilename('fullpath')),'launcher_arguments.mat'),'report','-v7');
end
end
