classdef bdDeadline < casadi.Callback
    % Mutable per-call wall deadline; graph can remain cached across tasks.
    properties
        clockHandle
        budget
        nx
        ng
        np
    end
    methods
        function self=bdDeadline(nx,ng,np)
            self@casadi.Callback(); self.nx=nx; self.ng=ng; self.np=np;
            self.clockHandle=tic; self.budget=inf;
            self.construct('bd_wall_deadline',struct());
        end
        function n=get_n_in(self), n=casadi.nlpsol_n_out(); end
        function n=get_n_out(self), n=1; end
        function name=get_name_in(self,i), name=casadi.nlpsol_out(i); end
        function name=get_name_out(self,i), name='stop'; end
        function s=get_sparsity_in(self,i)
            name=casadi.nlpsol_out(i);
            switch name
                case 'f', n=1;
                case {'x','lam_x'}, n=self.nx;
                case {'g','lam_g'}, n=self.ng;
                case 'lam_p', n=self.np;
                otherwise, n=0;
            end
            s=casadi.Sparsity.dense(n,1);
        end
        function s=get_sparsity_out(self,i), s=casadi.Sparsity.scalar(); end
        function result=eval(self,arg), result={double(toc(self.clockHandle)>=self.budget)}; end
    end
end
