function S = TS(S,u,AB,W)

% Author:       Mehran Attar - Montreal, Canada
% Written:      22-August-2024
% Last update:  --------------
% Last revision: ------------- 
%---------------------------------------------------------------
% This function simualtes the behavior of the data-driven tracking
% supervisor by computing the forward one step evolution of the system. 

%------------- BEGIN CODE -------------- 

u1 = zonotope(u,0*diag(ones(2,1)));
S = (AB * (cartProd(S,u1))) + W;

%------------- END CODE ----------------
end

