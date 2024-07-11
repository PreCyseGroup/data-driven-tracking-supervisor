
clc
clear all
close all
rand('seed',1);

% Discrete system x(k+1)=Ax(k)+Bu(k).

w = warning ('off','all');
rmpath('folderthatisnotonpath')
warning(w)
A = [1 4;0.8 0.5];
B = [0; 1];
C = eye(2);
D = 0;
% define continuous time system
sys_c = ss(A,B,C,D);
% convert to discrete system
samplingtime = 0.02;
sys_d = c2d(sys_c,samplingtime);

% defining system matrices 
Ad=sys_d.A;
Bd=sys_d.B;
Cd =sys_d.C;
Dd =sys_d.D;
dim_x = size(Ad,1);
dim_u = size(Bd,2);
model = LTISystem('A',Ad,'B',Bd);

% defining constraints 
X = zonotope(interval([-4;-4],[4;4])); % constraints on states
U = zonotope(0,6); % constraint on input
W = zonotope(zeros(dim_x,1),0.005*eye(dim_x));  % noise zonotope 
%% compute all possible A and B
sys = ss(Ad,Bd,Cd,Dd);  % define system
[V_AB,AB] = compute_AB(sys,X,U,W);
for i=1:size(V_AB,1)
    A_hat{i} = V_AB{i}(1:size(Ad,1),1:size(Ad,1));
    B_hat{i} = V_AB{i}(1:size(Ad,1),size(Ad,1)+1:size(Ad,1)+size(Bd,2));
end
%% Computing ROSC sets 

close all
X0 = zonotope(zeros(2,1),0.1*[1 -1 1;-1 1 1]);
% initialization of ROSC sets
Td{1} = X0;   % target set
N = 100;   % number of ROSC sets 
% temp_gen = 0.1*[2 1 2 1;2 1 1 -1;5 5 5 4]; % define a template for computing a zonotopic inner approximation of ROSC sets
temp_gen = [5 1 7;2 -1 3;5 1 -1];
temp_gen = 0.1*eye(3);

for step=1:N
    Td{step+1} = compute_intersec(compute_presets_approx(Td{step}.mptPolytope.P, ...
        W.mptPolytope.P, U.mptPolytope.P, X.mptPolytope.P, A_hat, B_hat));
    Td{step+1} = poly_approx(Td{step+1}, size(temp_gen,2), temp_gen);
    Td_aug{step+1} = Td{step+1};
    Td{step+1} = project(Td_aug{step+1},[1 2]);
    step
    
    handleData = plot(Td{step+1});
    hold on
    pause(0.1)
end



