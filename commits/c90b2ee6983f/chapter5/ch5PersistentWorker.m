function summary=ch5PersistentWorker(stage,selection)
% Retain model/native functions across episodes, never the episode state.
if nargin<2,selection=struct();end
assert(ismember(string(stage),["development","formal"]));
setappdata(0,'ch5ReuseEnabled',true);
cleanup=onCleanup(@finish);
selection.keep_model=true;
summary=ch5RunStage(stage,selection);
end
function finish()
if isappdata(0,'ch5ReuseEnabled'),rmappdata(0,'ch5ReuseEnabled');end
ch5ReleaseEpisodeData();
end
