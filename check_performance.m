function out = check_performance(X_1,index,V)

for i=1:size(index)
    if X_1.mptPolytope.P.contains(V{index(i)}) == 1
        out = 1;
        break
    else
        out = 0;
    end
end

