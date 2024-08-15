function command = one_step_ctrl(dim_u,x_curr,aug_set,index,v)

% Author:       Mehran Attar
% Written:      08-March-2023
% Last update:
% Last revision:---
% This function computes the ST-MPC control commands by leveraging the
% data-driven ROSC sets
      
%------------- BEGIN CODE --------------

u = sdpvar(dim_u,1);
const = aug_set{v,index}.mptPolytope.P.A * [x_curr;u] <= aug_set{v,index}.mptPolytope.P.b;
% const = aug_set{index}.A * [x_curr;u] <= aug_set{index}.b;
obj = norm(u,2);
opt = sdpsettings('solver','sedumi','verbose',0);
diagnostics=optimize(const,obj,opt);
command = value(u);

end

%------------- END CODE --------------