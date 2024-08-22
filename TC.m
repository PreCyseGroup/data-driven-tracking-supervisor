function u = TC(x_prime,ss_input,K)
% Author:       Mehran Attar - Montreal, Canada
% Written:      22-August-2024
% Last update:  --------------
% Last revision: ------------- 
%---------------------------------------------------------------
% This function computes the control signal $u_k$ using the received state 

%------------- BEGIN CODE -------------- 
u = -K*x_prime + ss_input;
u =  min(max(u, [-2;-10]), [2;10]);   % saturation 
%------------- END CODE ----------------
end

