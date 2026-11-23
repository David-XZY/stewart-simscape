function render_training(runFolder)
% Plot recorded validation loss; never infer unrecorded training loss.
asset = jsondecode(fileread(fullfile(runFolder,'training','model.json')));
assert(strcmp(asset.status,'trained'),'The model has not completed training.');
lines = regexp(strtrim(fileread(fullfile(runFolder,'training','training.jsonl'))),'\r?\n','split');
epochs = zeros(numel(lines),1); values = epochs;
for i=1:numel(lines)
    row = jsondecode(lines{i});
    epochs(i) = row.epoch+1; values(i) = row.validationNoiseMSE;
end
assert(all(isfinite(values)) && all(values>=0),'Invalid recorded loss.');
assert(all(diff(epochs)>0),'Repeated or out-of-order training epochs.');
fig = figure('Visible','off','Color','w','Position',[100 100 1050 480]);
cleanup = onCleanup(@() close(fig)); %#ok<NASGU>
plot(epochs,values,'LineWidth',1.4); hold on; grid on;
yline(asset.bestValidationNoiseMSE,'--','Exported best validation weight','LabelHorizontalAlignment','left');
[minimum,index] = min(values);
if abs(minimum-asset.bestValidationNoiseMSE)<1e-9
    plot(epochs(index),minimum,'o','MarkerFaceColor',[0.85 0.33 0.1],'MarkerSize',7);
end
xlabel('Epoch (one-based)'); ylabel('Validation noise-prediction MSE');
[~,runName] = fileparts(runFolder);
title(sprintf('%s: recorded validation loss (%d completed epochs)',runName,asset.epochsCompleted),'Interpreter','none');
folder = fullfile(runFolder,'figures'); if ~exist(folder,'dir'), mkdir(folder); end
print(fig,fullfile(folder,'training_validation.png'),'-dpng','-r160');
end
