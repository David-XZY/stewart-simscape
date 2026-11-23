function r=ch5ReducedPrepareV43(index,mode)
% Reuse the installed physical model; change only the development horizon.
[c,~]=ch5Init();
if evalin('base','exist(''ch5Runtime'',''var'')')==1
    template=evalin('base','ch5Runtime');
    if template.c.H~=c.H
        assert(template.c.Ts==c.Ts&&template.plant.Ts==c.Ts,'ch5:InvalidTemplate','Physical sampling changed.');
        template.c.H=c.H;template.c.version=[c.version,'-speed-reduced-v1'];assignin('base','ch5Runtime',template);
    end
end
r=ch5ReducedPrepare(index,mode);r.c.H=30;r.c.version=[c.version,'-speed-reduced-v43-H30'];
end
