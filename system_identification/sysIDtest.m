% Author:       Mehran Attar - Montreal, Canada
% Written:      15-May-2026
% Last update:  --------------
% Last revision: -------------
%---------------------------------------------------------------


%------------- BEGIN CODE --------------


clc; 
clear all; 
close all;

% System definition
A = [0.9719 0.0013; 0.0340 0.8628];
B = [-0.0839 0.0232; 0.0761 0.4144];
C = [1 0; 0 1];
D = 0;
bound = 0.001;  % disturbance bound 
X_zono = zonotope(interval([-10; -30], [10; 30]));
U_zono = zonotope(interval([-2; -10], [2; 10]));
W_zono = zonotope(zeros(2,1), bound * eye(2));
Ts = 1;  % Sample time
N = 1000; % Number of samples

% Zonotopic constraints (bounds)
X_min = [-10; -30];
X_max = [10; 30];
U_min = [-2; -10];
U_max = [2; 10];


% Data initialization
nx = size(A, 1);
nu = size(B, 2);
ny = size(C, 1);

X = zeros(nx, N+1);
U = zeros(nu, N);
Y = zeros(ny, N);
X_data(:,1) = [0.1;0.1];
% Generate data with bounded disturbance
for k = 1:N
    u_k = U_zono.randPoint;  % Random input within bounds
    w_k = W_zono.randPoint;              % Bounded disturbance
    x_next = A*X_data(:,k) + B*u_k + w_k;

    % Clip to state bounds
    x_next = min(max(x_next, X_min), X_max);

    X_data(:,k+1) = x_next;
    U(:,k) = u_k;
%     Y(:,k) = C*x_next; % or C*X(:,k+1)
end

% Format data for identification
data_id = iddata(X_data(:,1:N)', U', Ts);

% System identification using n4sid
sys_order = 2; % Known or estimated
sys_est = n4sid(data_id, sys_order, 'Form', 'canonical');

A_est = sys_est.A;
B_est = sys_est.B;

%% Forward reachable sets computation
close all

x_init = [10; 6];
u_init = [0.1;-9];
% x_init = X_zono.randPoint;
% u_init = U_zono.randPoint;

R1 = A*x_init + B*u_init + W_zono;  % Forward reachable sets - model-based approach
handle_model = plot(R1,[1 2],'color','k','LineWidth',2)
hold on
R2 = A_est*x_init + B_est*u_init + W_zono;   % Forward reachable sets - system identification approach 
handle_idn = plot(R2,[1 2],'color','r','LineWidth',2)

%% Forward reachable sets - data-driven approach

hold on 
sys = ss(A,B,C,D);  % defining system
initpoints = 10;
steps = 5; 
[AB,X_0T,U_full] = compute_AB(sys,X_zono,U_zono,W_zono,initpoints,steps);   % computing system matrices that consistant with data

state = zonotope(x_init,0*diag(ones(2,1)));
ctr = zonotope(u_init,0*diag(ones(2,1)));
R3 = AB * (cartProd(state, ctr))+ W_zono;

handle_data = plot(R3,[1 2],'color','b','LineWidth',2);

legend([handle_model, handle_idn, handle_data],'Model-based approach','Sys Identification approach',...
    'Data-driven (over-approximation)', 'FontSize', 10);

title(sprintf('forward reachable set $\\mathcal{W}(0, %.3f)$', bound), ...
      'interpreter', 'latex', 'FontSize', 14);


print -depsc -tiff -r300 -painters comparison_id.eps


%------------- END CODE --------------
