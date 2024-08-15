
% Author:       Mehran Attar
% Written:      10-December-2023
% Last update:  --------------
% Last revision: 10-December-2023

%------------- BEGIN CODE --------------

clc
clear all
close all
rand('seed',1);

% Discrete system x(k+1)=Ax(k)+Bu(k).

w = warning ('off','all');
rmpath('folderthatisnotonpath')
warning(w)

% defining system matrices 
% defining system matrices
A = [1 -1;1 1];
B = [1;0];
C=eye(2);
D=0;

dim_x = size(A,1);
dim_u = size(B,2);

% defining system constraints


X = zonotope(interval([-1;-1],[1;1]));  % constraints on states
U = zonotope(interval([-1.2],[1.2])); % constraints on inputs
W = zonotope(zeros(dim_x,1),0.001*eye(dim_x));  % noise zonotope
model = LTISystem('A',A,'B',B);
X0 = zonotope([0.1;0.1],0.03*[0 -1 1;-1 1 1]);

% Visualization of the RCI set and it's zonotopic over-approximation set
figure;
plot(X0,[1 2],'r--','LineWidth',1)
hold on
% plot(T0,'Alpha',0.3,'color','g');
xlabel('$x_1$','interpreter','latex','FontSize',20)
ylabel('$x_2$','interpreter','latex','FontSize',20)
legend(['RCI set based on model'],['Zonotopic over-approximation of RCI '])
box off
%% compute all possible A and B
sys = ss(A,B,C,D);  % define system
[V_AB,AB] = compute_AB(sys,X,U,W);
for i=1:size(V_AB,1)
    A_hat{i} = V_AB{i}(1:size(A,1),1:size(A,1));
    B_hat{i} = V_AB{i}(1:size(A,1),size(A,1)+1:size(A,1)+size(B,2));
end
%% Computing ROSC sets 

close all
% X0 = zonotope([0.2;0.1],0.005*[0 -1 1;-1 1 1]);

% X0 = zonotope([0.2;0.1],0.005*[0.1 -1 2;-1 2 -1]);
% hold on 
% border1 = zonotope(interval([0.1;-0.15],[0.3;0.3]));
% plot(border1)
% plot(X0,[1 2],'r')
%
% hold on 
% X0 = zonotope(zeros(2,1),0.01*[1 -2 1;-2 1 2]);
% initialization of ROSC sets
Td1{1} = X0;   % target set
N = 80;   % number of ROSC sets 
% temp_gen = 0.1*[1 -1 1;0 2 1]; % define a template for computing a zonotopic inner approximation of ROSC sets
% temp_gen = 0.1*[0 -2 1;-2 1 2];
% temp_gen = [0.1 -1 2;-1 2 -1]; 
temp_gen = 0.1*[1 -1;0 2];
% temp_gen = 0.1*[0 -2 1;2 -1 -1];
%
for step=1: N
    for j=1:size(A_hat,2)
        presets{j} = (inv(A_hat{j})*((Td1{step}.mptPolytope.P - W.mptPolytope.P)...
            + (-B_hat{j}*U.mptPolytope.P)));
    end
    %
    Td1{step+1} = compute_intersec(presets)& X.mptPolytope.P;   
    [Td1{step+1},alpha] = poly_approx(Td1{step+1}, size(temp_gen,2), temp_gen); % computing zonotopic inner approximation of ROSC sets
%     Td1{step+1} = reduce(Td1{step+1},'girard',200);
    Td_aug1{step+1} = zonotope([Td1{step+1}.center;U.center],[Td1{step+1}.generators;U.generators]); % Augmented ROSC sets 
    step
    % Visualization of ROSC sets 
    handleData = plot(Td1{step+1});
    hold on
    pause(0.1)
end

%% Saving ROSC sets 
% save('Td')
% save('Td_aug')
