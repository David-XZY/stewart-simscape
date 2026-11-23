function report=ch5SpeedConditionStudyV19()
clear ch5ReducedSolveV19
[~,root]=ch5Init();base=fullfile(root,'chapter5','artifacts','reduced_probe');directory=fullfile(base,'conditioning_study_v19');
assert(~isfolder(directory),'ch5:ExistingStudy','Retain all attempts.');mkdir(directory);
source=load(fullfile(base,'solver_study_v16','study.mat'),'report');report=struct('identity',ch5ReducedIdentity(),'pid',feature('getpid'),'rows',{{}});
for index=1:numel(source.report.matrices)
    sample=source.report.matrices{index};
    for backend=["whiten-active","whiten-null-active"]
        times=[];
        for repeat=0:2
            [delta,info]=ch5ReducedSolveV19(sample.linear,backend);if repeat>0,times(end+1)=info.total_seconds;end
        end
        row=struct('mode',sample.mode,'time',sample.time,'backend',char(backend),'seconds',times,'solve',info,'delta',delta);
        report.rows{end+1}=row;
        fprintf('Condition study %s t%.1f %s %.1fms flag%d iter%d constraints%d/%d obj%.5g\n', ...
            sample.mode,sample.time,backend,1000*median(times),info.exitflag,info.iterations,info.inequalities,numel(sample.linear.b),info.objective);
    end
end
save(fullfile(directory,'study.mat'),'report','-v7');
fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
