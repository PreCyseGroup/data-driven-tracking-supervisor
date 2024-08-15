function J = cal_performance(V,allowed_index,ref_index,X_1,W,I)

J = 0;
for i=1:max(size(allowed_index))
% %     vol = vol + (volume(X_1.mptPolytope.P & V{allowed_index(i)})/volume(X_1.mptPolytope.P));
    J = J + (volume(X_1.mptPolytope.P & V{allowed_index(i)})/volume(X_1.mptPolytope.P))*I{allowed_index(i),ref_index};
end
%
% J = 0;
% for i=1:max(size(allowed_index))
%     J = J + vol*I{allowed_index(i),ref_index};
% end
end

