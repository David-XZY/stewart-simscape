function report=ch5CheckNativeCollision()
[c,root]=ch5Init();[m,s]=ch5Physics(c);collision=ch5CollisionModel(m,s,c);
report=struct('version',c.version,'passed',false,'maximum_error',0);
for nt=[9,10]
    traceTime=linspace(0,c.Ts,nt);r=struct('c',c,'collision',collision,'plant',struct('traceTime',traceTime));
    times=reshape(traceTime(:)+(0:c.H-1)*c.Ts,1,[]);n=numel(times);
    q=repmat(m.qHome+[0;0;.15;zeros(3,1)],1,n);q(1,:)=.03*sin(times);q(6,:)=.06*cos(times);
    o=zeros(22,c.max_obstacles);o(7:15,:)=repmat(reshape(eye(3),9,1),1,c.max_obstacles);
    for j=[1,3]
        o(1:3,j)=[.8;(-1)^j*.4;1.2];o(4:6,j)=[.04;.1;.2];
        o(7:15,j)=reshape(ch5Rotation([.1;-.15;.2*j]),9,1);o(16:18,j)=[-.01;.02;.003];o(22,j)=1;
    end
    expected=inf;
    for k=1:n
        future=o;future(1:3,:)=o(1:3,:)+times(k)*o(16:18,:);
        gaps=full(collision.value(q(:,k),future));expected=min(expected,min(gaps([1,3])));
    end
    actual=ch5NativeCollisionAudit(q,o,r);error=abs(actual-expected);
    report.maximum_error=max(report.maximum_error,error);assert(error<1e-10,'ch5:CollisionBatchMismatch','Native inner-grid collision audit differs.');
    fprintf('Native collision grid %d: error %.3g\n',n,error);
end
report.passed=true;save(fullfile(root,'chapter5','artifacts','validation','native_collision.mat'),'report');
end
