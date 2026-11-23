function options=ch5ProjectionJitOptions()
% Large projection Jacobian functions need native optimization at run time.
% The private compiler uses O1 for large functions and O2 for small ones;
% baseline compilation and its admitted teacher source remain unchanged.
options=ch5JitOptions();
options.jit_options.compiler=strrep(options.jit_options.compiler,'compile.py','compile_projection.py');
end
