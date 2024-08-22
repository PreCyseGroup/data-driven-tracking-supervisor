function J = performance_cal(V,ref_index,X_1,I)
% Author:       Mehran Attar - Montreal, Canada
% Written:      22-August-2024
% Last update:  --------------
% Last revision: ------------- 
%---------------------------------------------------------------
% This function computes the tracking performance based on equation (32) in the corresponding reference  

%------------- BEGIN CODE -------------- 
J = 0;
for i=1:5
    J = J + (volume(X_1.mptPolytope.P & V{i})/volume(X_1.mptPolytope.P))*I{i,ref_index};
end
%
%------------- END CODE ----------------
end

