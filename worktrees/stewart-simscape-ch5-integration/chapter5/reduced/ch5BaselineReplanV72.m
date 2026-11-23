function [s,replanned,entry]=ch5BaselineReplanV72(s,q,qd,goal,o,r,time)
% Original IHSID is called whenever the retained task plan fails revalidation.
[retained,entry]=ch5BaselineRetainV72(s,q,qd,goal,o,r,time);
if retained,s.attemptTime=time;replanned=false;return;end
[s,replanned,entry]=ch5BaselineReplanV67(s,q,qd,goal,o,r,time);
end
