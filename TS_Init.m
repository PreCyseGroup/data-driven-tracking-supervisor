function R1 = TS_Init(x,u,AB,W)

% Author:       Mehran Attar - Montreal, Canada
% Written:      22-August-2024
% Last update:  --------------
% Last revision: ------------- 
%---------------------------------------------------------------
% This function initialize the tracking supervisor module.  

%------------- BEGIN CODE --------------
x1 = zonotope(x,0*diag(ones(2,1)));
u1 = zonotope(u,0*diag(ones(2,1)));
R1 = (AB * (cartProd(x1,u1))) + W;
%------------- END CODE ----------------
end

