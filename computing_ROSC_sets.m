
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
X = zonotope(interval([-2.5;-10],[2.5;10])); % constraints on states
U = zonotope(0,5); % constraint on input
W = zonotope(zeros(dim_x,1),0.001*eye(dim_x));  % noise zonotope 

% Design Terminal RCI region for model-based 

% quadratic penalties
model.x.penalty = QuadFunction([2 0; 0 1]);
model.u.penalty = QuadFunction(eye(1));

% get the LQR feedback
K = model.LQRGain;

%Find the terminal region
Acl = Ad+(Bd*K);

%compute RCI region
alpha = 0.7;
T0 = computeRPI(Acl,alpha,W.mptPolytope.P);

% Considering an over-approximation of the RCI set
%%
close all
X0 = zonotope(zeros(2,1),0.02*[4 1 -2;-2 1 1]);

% Visualization of the RCI set and it's zonotopic over-approximation set
figure;
plot(X0,[1 2],'r--','LineWidth',1)
hold on
plot(T0,'Alpha',0.3,'color','g');
xlabel('$x_1$','interpreter','latex','FontSize',20)
ylabel('$x_2$','interpreter','latex','FontSize',20)
legend(['RCI set based on model'],['Zonotopic over-approximation of RCI '])
box off
%% compute all possible A and B
sys = ss(Ad,Bd,Cd,Dd);  % define system
[V_AB,AB] = compute_AB(sys,X,U,W);
for i=1:size(V_AB,1)
    A_hat{i} = V_AB{i}(1:size(Ad,1),1:size(Ad,1));
    B_hat{i} = V_AB{i}(1:size(Ad,1),size(Ad,1)+1:size(Ad,1)+size(Bd,2));
end
%% Computing ROSC sets 

close all

% initialization of ROSC sets
Td{1} = X0;   % target set
N = 100;   % number of ROSC sets 
temp_gen = 0.1*[-1 1 -1;0 -1 1]; % define a template for computing a zonotopic inner approximation of ROSC sets
%
for step=1: N
    for j=1:size(A_hat,2)
        presets{j} = (inv(A_hat{j})*((Td{step}.mptPolytope.P - W.mptPolytope.P)...
            + (-B_hat{j}*U.mptPolytope.P)));
    end
    %
    Td{step+1} = compute_intersec(presets)& X.mptPolytope.P;   
    [Td{step+1},alpha] = poly_approx(Td{step+1}, size(temp_gen,2), temp_gen); % computing zonotopic inner approximation of ROSC sets
    Td_aug{step+1} = zonotope([Td{step+1}.center;U.center],[Td{step+1}.generators;U.generators*ones(1,3)]); % Augmented ROSC sets 
    step
    % Visualization of ROSC sets 
    handleData = plot(Td{step+1});
    hold on
    pause(0.1)
end
%%
Td = load('Td.mat').Td;   % ROSC sets
% V_AB = load('V_AB.mat').V_AB;  % set of vertices \mathcal{V}_{AB} as a matrix zonotope
% AB = load('AB.mat').AB;   % set of system matrices \mathcal{M}_{AB} as a matrix zonotope
Td_aug = load('Td_aug.mat').Td_aug;  % set of augmented ROSC sets
for i=1: 80
    plot(Td{i})
    hold on 
end
%% Saving ROSC sets 
save('Td')
save('Td_aug')
