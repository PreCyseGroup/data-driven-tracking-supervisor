
% Author:       Mehran Attar - Montreal, Canada
% Written:      22-August-2024
% Last update:  --------------
% Last revision: -------------
%---------------------------------------------------------------
% Purpose:
% This code implements a data-driven control architecture designed to preserve safety and tracking performance 
% in constrained Cyber-Physical Systems (CPS) under network attacks. 
% The architecture uses data-driven techniques to maintain system robustness and resilience against attacks, 
% while adhering to system constraints. 
% The core algorithms implemented here enable monitoring, analysis, and adjustment of system behavior to uphold 
% optimal performance and safety standards. Moreover, the results are
% compared with the setups without tracking supervisor module and absence
% of attacks. 

%------------- BEGIN CODE --------------

clc
clear all
close all

w = warning ('off','all');
rmpath('folderthatisnotonpath')
warning(w)

% defining system matrices 
A = [0.9719 0.0013;0.0340 0.8628];  % System dynamics matrix
B = [-0.0839 0.0232;0.0761 0.4144];        % Input matrix
C = [1, 0; 0, 1];  % Output matrix
D = 0;
dim_x = size(A,1);   % state dimension 
dim_u = size(B,2);   % control input dimension 
sys = ss(A,B,C,D);

% defining constraints on input, states and disturbance
X = zonotope(interval([-10;-30],[10;30])); % constraints on states
U = zonotope(interval([-2;-10],[2;10])); % constraint on input
W = zonotope(zeros(2,1),0.001*eye(2));  % noise zonotope

% defining tracking controller parameters 
R = 1*eye(1);
N = 0;
Q = 1*eye(2);
K = dlqr(A,B,Q,R);

% loading ROSC sets & Voronoi regions and tracking
% controller DoA 

Td1 = load('Td1').Td1;    % ROSC sets 
Td1_aug = load('Td1_aug.mat').Td1_aug;   % augment ROSC sets 
V = load('V').V;   % voronoi regions 
Td_f = load('Td_f').Td_f;  % tracking controller DoA 


initpoints = 2;
steps = 2;
AB = compute_AB(sys,X,U,W,initpoints,steps);   % computing system matrices that consistant with data

% simulation settings
sim_time = 600;    % simulation time
sys = ss(A,B,C,D);  % defining system


% defining reference signal
for i=1:sim_time
    if i<60
        r(:,i) = [-1;2];
    elseif i>=60 & i<200
        r(:,i) = [9; 20];
    elseif i>=200 & i<300
        r(:,i) = [-6;-7];
    elseif i>=300 & i<400
        r(:,i) = [-7;-15];
    elseif i>=400 & i<500
        r(:,i) = [3;27];
    else
        r(:,i) = [4;25];
    end
end
%
% defining FDI attack on the measurement channel
for i=1:sim_time+1
    if i>=60 && i<=110
        y_a(:,i)=0.01*[(i-59);(i-59)];
        attack(i)=1;
    elseif i>=200 && i<=220
        y_a(:,i)=0.08*[(i-199);(i-199)];
        attack(i)=1;
    elseif i>=240 && i<=260
        y_a(:,i)=[0.1*(i-239);0.1*(i-239)];
        attack(i)=1;
    elseif i>400 && i<=420
        y_a(:,i)=[0.1*(i-399);0.1*(i-399)];
        attack(i)=1;
    else
        y_a(:,i)=[0;0];
        attack(i)=0;
    end
end

% defining FDI attack on the actuation channel
for i=1:sim_time
    if i>=150 && i<160
        u_a(:,i)=[0;0];
    else
        u_a(:,i)=[0;0];
    end
end

set_num = 5;   % vonornoi sets 

% initialization of the variables 
alarm(1) = 0;
x(:,1) = [0.01;-0.01];  % initial state
x_data(:,1) = [0.01;-0.01];
x_prime(:,1) = [0.01;-0.01]; % initial state
x_data_prime(:,1) = [0.01;-0.01];
flag = 0;
ignore = 0;
emergency(1) = 0;
flag1 = 0;
ignore1 = 0;
emergency1(1) = 0;
x_hat = zeros(2,600);   % estimated state from the outer-approximated RORS sets 
x_pre_alarm = {};


% computing equlibirium points
for i=1:sim_time
    ss_input(:,i) = pinv(C * inv(eye(size(A)) - A + B * K) * B) * r(:,i);
end
%
% Visualization of the state space 
f = figure;
ax = axes;
f.Position = [700 70 800 700]

plot(x(1,1),x(2,1),'*','MarkerSize',4,'MarkerEdgeColor','k')
hold on
plot(x_data(1,1),x_data(2,1),'*','MarkerSize',4,'MarkerEdgeColor','r')
index = 1;
projectedDims = {[1 2]};
hold on
for i=1:5
    plot(V{i},'Alpha',0.01,'color','white','EdgeColor','r','LineWidth',2)
    hold on
end
hold on
plot(Td_f)
hold on
xlabel('$x_1$','interpreter','latex','FontSize',24)
ylabel('$x_2$','interpreter','latex','FontSize',24)
r1 = [-1;9;5;-9;-6];
r2 = [2;25;5;-25;-15];
p1 = [4;-6;0;6;-4];
p2 = [15;15;0;-20;-20];
for i=1:set_num
    plot(p1(i),p2(i),'r*')
    hold on
end

r1 = [-1;9;-6;-7;3;4];
r2 = [2;20;-7;-15;27;25];
for i=1:6
    plot(r1(i),r2(i),'b*','MarkerSize',5)
    hold on
end

xlim([-11 11]);
ylim([-31 31]);
text(0.103,-0.1,'$x_{0}$','FontSize',17,'interpreter','latex')
hold on
annotation('textbox',...
    [0.81975 0.855142857142857 0.04 0.0422857142857144],'String',{'V_1'},...
    'FitBoxToText','off');
annotation('textbox',...
    [0.18425 0.854571428571429 0.04 0.0422857142857144],'String',{'V_2'},...
    'FitBoxToText','off');
annotation('textbox',...
    [0.81675 0.534 0.04 0.0422857142857143],'String',{'V_3'},...
    'FitBoxToText','off');
annotation('textbox',...
    [0.81475 0.338571428571429 0.04 0.0422857142857146],'String',{'V_4'},...
    'FitBoxToText','off');
annotation('textbox',...
    [0.18875 0.340857142857143 0.04 0.0422857142857143],'String',{'V_5'},...
    'FitBoxToText','off');

% Computing performance index (see equation (30) in the corresponding
% reference 
for i=1:5
    for j=1:5
        I{i,j} = sqrt((p1(i)-p1(j))^2 + (p2(i)-p2(j))^2);
    end
end

init_ts = 0;  % tracking supervisor initialization flag
ts_flag = 0;  % tracking supervisor activation flag 
   
D_delay = 5;  % anomaly detector detection delay 

fprintf('=========================================================================================')
fprintf('\n')
fprintf(' =====  simulating the proposed method using the tracking supervisor module =============')
fprintf('=========================================================================================')
fprintf('\n')
pause(5)
% simulation of the system -- presence of attack on the measurement channel
for k=1:sim_time
    pause(0.1)
    fprintf('time = %d\n',k)
    fprintf(' \n')
    index_x{k} = partition_index(V,x(:,k));
    index_r{k} = partition_index(V,r(:,k));
    
    
    u(:,k) = TC(x(:,k),ss_input(:,k),K);
    
    
    if alarm(k) == 0
        ts_flag = 0;
    end
    
    
    if alarm(k) == 1 & flag == 0
        if init_ts == 0
            fprintf('%d\n activating TS ')
            fprintf(' \n')
            R1{k} = TS_Init(x(:,k-D_delay),u(:,k-D_delay),AB,W);
            fprintf(' %d\n TS has been initialized')
            fprintf(' \n')
            init_ts = 1;
            S{k} = R1{k};
            ts_flag = 1;
        end
    end
    if init_ts == 1 & ts_flag == 1
        fprintf('TS is running %d\n')
        fprintf(' \n')
        x_hat(:,k) = S{k}.randPoint;
        u_open_loop(:,k) = TC(x_hat(:,k),ss_input(:,k),K);
        S{k+1} = TS(S{k}, u_open_loop(:,k), AB, W);
        S{k+1} = reduce(S{k+1},'girard',20);
        J{k+1} = performance_cal(V,index_r{k},S{k+1},I);
        plot(S{k+1})
        
        % evaluating the tracking performance 
        if J{k+1} > J{k}
            flag = 1;
        end
        if Td_f.contains(S{k}) == 0
            flag = 1;
            init_ts = 0;
            ts_flag = 0;
        else
            ts_flag = 1;
        end
        
    end
    
    hold on
    if ts_flag == 1
        state = zonotope(x(:,k),0*diag(ones(2,1)));
        ctr = zonotope(u_open_loop(:,k),0*diag(ones(2,1)));
        S_alarm{k} = AB * (cartProd(state, ctr))+ W;
    else
        state = zonotope(x(:,k),0*diag(ones(2,1)));
        ctr = zonotope(u(:,k),0*diag(ones(2,1)));
        S_alarm{k} = AB * (cartProd(state, ctr))+ W;
    end
    v = partition_index(V,x(:,k));
    
    if flag == 1 && Td1{v,1}.contains(x(:,k)) == 1
        flag = 0;
        ignore = 1;
    else
        ignore = 0;
    end
    
    % safety check
    [x_plus{k},safety(k)] = data_driven_safety_guard(u(:,k),...
        x(:,k),U,Td_f,AB,W);
    %
    if safety(k) == 1
        flag = 1;
    end
    %
    % Swithcing between emergency controller and tracking controller
    if flag == 1
        index(k) = set_index(x(:,k),Td1,v); % computing set membership index
        fprintf('EM controller is active')
        fprintf(' \n')
        u_p(:,k) = one_step_ctrl(2, x(:,k), Td1_aug, index(k),v); % computing D-ST-MPC controller
        emergency(k) = 1; % emergency controller activation singal
    else
        if ts_flag == 1 & flag == 0
            u_p(:,k) = u_open_loop(:,k); % applying tracking controller to the plant
            fprintf('Open-loop controller is active')
            fprintf(' \n')
            emergency(k) = 0; % emergency controller activation singal
        else
            u_p(:,k) = u(:,k); % applying tracking controller to the plant
            fprintf('TC controller is active')
            fprintf(' \n')
            emergency(k) = 0; % emergency controller activation singal
            init_ts = 0;
        end
    end
    
    % computing the system evolution
    x(:,k+1) = A*x(:,k) + B*u_p(:,k) + randPoint(W);
    x_prime(:,k+1) = x(:,k+1) + y_a(:,k+1);
    handle_state = plot(x(1,k),x(2,k),'o','MarkerSize',2,'MarkerEdgeColor','k','MarkerFaceColor','k');
    hold on

    alarm(k+1) = detector_data_driven(x_prime(:,k+1),S_alarm{k});
end

%% configuration proposed in:
% "A Data-Driven Safety Preserving Control Architecture for Constrained Cyber-Physical Systems"
% by "Mehran Attar and Walter Lucia" 
fprintf('=========================================================================================')
fprintf('\n')
fprintf('======= simulating the proposed method without tracking module ==========================')
fprintf('\n')
fprintf('=========================================================================================')
pause(5)
hold on
for k=1:sim_time
    
    fprintf('time = %d\n', k)
    fprintf('\n')
    % computing data-driven tracking controller
    ctr_data(:,k) = -K*x_data_prime(:,k) + ss_input(:,k);
    ctr_data(:,k) =  min(max(ctr_data(:,k), [-2;-10]), [2;10]);
    
    % computing the one-step evolution set, \hat{\mathcal{R}}_k
    x1 = zonotope(x_data_prime(:,k),0*diag(ones(dim_x,1)));
    u = zonotope(ctr_data(:,k),0*diag(ones(dim_u,1)));
    x_pre_data{k} = AB * (cartProd(x1,u))+ W;
    
    % attack on actuation (Note: in this scenario we dont have any attacks
    % on the actuation channel)
    ctr_data_prime(:,k) = ctr_data(:,k) + u_a(:,k);
    
    v = partition_index(V,x_data(:,k));
    if flag == 1 & Td1{v,1}.contains(x_data(:,k)) == 1
        flag = 0;
        ignore = 1;
    else
        ignore = 0;
    end
    %
    
    % safety check
    [x_plus_data{k},safety_data(k)] = data_driven_safety_guard(ctr_data_prime(:,k),...
        x_data(:,k),U,Td_f,AB,W);
    
    
    if safety_data(k) == 1
        flag = 1;
    end
    
    % Swithcing between emergency controller and tracking controller
    if flag == 1
        index_data(k) = set_index(x_data(:,k),Td1,v); % computing set membership index
        u_ver(:,k) = one_step_ctrl(2, x_data(:,k), Td1_aug, index_data(k),v); % computing D-ST-MPC controller
        emergency(k) = 1; % emergency controller activation singal
        fprintf('emergency controller is active')
        fprintf('\n')
    else
        u_ver(:,k) = ctr_data_prime(:,k); % applying tracking controller to the plant
        emergency(k) = 0; % emergency controller activation singal
        fprintf('tracking controller is active')
        fprintf('\n')
    end
    
    % computing the system evolution
    x_data(:,k+1) = A*x_data(:,k) + B*u_ver(:,k) + randPoint(W);
    
    % attack on measurement channel
    x_data_prime(:,k+1) = x_data(:,k+1) + y_a(:,k);
    
    % visualization of the system evolution
    handle_old = plot(x_data(1,k),x_data(2,k),'o','MarkerSize',2,'MarkerEdgeColor','r','MarkerFaceColor','r');
    hold on

    % anomaly detector
    alarm_data(k+1) = detector_data_driven(x_data_prime(:,k+1),x_pre_data{k});
    
    % send a signal from detector to plant in the case of attack on the
    % measurement channel
    if alarm_data(k+1) == 1
        flag = 1;
    end
    %
    if norm(y_a(:,k))==[0;0]
        attack(k)=false;
    else
        attack(k)=true;
    end
    pause(0.1)
   
end

hold on
y_a = [];
x_w_prime = [0;0]
x_w = [0;0];
for i=1:sim_time
    y_a(:,i)=[0;0];
end
%% Configuration in the absence of attacks 
fprintf('=========================================================================================')
fprintf('\n')
fprintf('==== simulating the dynamical behavior of the system in the absence of cyber attacks === ')
fprintf('\n')
fprintf('=========================================================================================')

pause(5)

hold on
for k=1:sim_time
    
    fprintf('time = %d\n', k)
    fprintf('\n')
    
    % computing data-driven tracking controller
    ctr_data(:,k) = -K*x_w_prime(:,k) + ss_input(:,k);
    ctr_data(:,k) =  min(max(ctr_data(:,k), [-2;-10]), [2;10]);
    
    % computing the one-step evolution set, \hat{\mathcal{R}}_k
    x1 = zonotope(x_w_prime(:,k),0*diag(ones(dim_x,1)));
    u = zonotope(ctr_data(:,k),0*diag(ones(dim_u,1)));
    x_pre_data{k} = AB * (cartProd(x1,u))+ W;
    
    % attack on actuation (Note: in this scenario we dont have any attacks
    % on the actuation channel)
    ctr_data_prime(:,k) = ctr_data(:,k) + u_a(:,k);
    
    v = partition_index(V,x_w(:,k));
    if flag == 1 & Td1{v,1}.contains(x_w(:,k)) == 1
        flag = 0;
        ignore = 1;
    else
        ignore = 0;
    end
    %
    
    % safety check
    [x_plus_data{k},safety_data(k)] = data_driven_safety_guard(ctr_data_prime(:,k),...
        x_w(:,k),U,Td_f,AB,W);
    
    
    if safety_data(k) == 1
        flag = 1;
    end
    
    % Swithcing between emergency controller and tracking controller
    if flag == 1
        index_data(k) = set_index(x_w(:,k),Td1,v); % computing set membership index
        u_ver(:,k) = one_step_ctrl(2, x_w(:,k), Td1_aug, index_data(k),v); % computing D-ST-MPC controller
        emergency(k) = 1; % emergency controller activation singal
    else
        u_ver(:,k) = ctr_data_prime(:,k); % applying tracking controller to the plant
        emergency(k) = 0; % emergency controller activation singal
    end
    
    % computing the system evolution
    x_w(:,k+1) = A*x_w(:,k) + B*u_ver(:,k) + randPoint(W);
    
    % attack on measurement channel
    x_w_prime(:,k+1) = x_w(:,k+1) + y_a(:,k);
    
    % visualization of the system evolution
    handle_w_attack = plot(x_w(1,k),x_w(2,k),'o','MarkerSize',2,'MarkerEdgeColor',[0.6784    0.8392    0.1020],...
        'MarkerFaceColor',[0.6784    0.8392    0.1020]);
    hold on
    
    % anomaly detector
    alarm_data(k+1) = detector_data_driven(x_w_prime(:,k+1),x_pre_data{k});
    
    % send a signal from detector to plant in the case of attack on the
    % measurement channel
    if alarm_data(k+1) == 1
        flag = 1;
    end
    %
    if norm(y_a(:,k))==[0;0]
        attack(k)=false;
    else
        attack(k)=true;
    end
    pause(0.1)
    
end
%% state trajectories for different setups

f = figure
f.Position = [700 70 600 400]

f1 = subplot(2,1,1)
f1.InnerPosition = [0.096,0.58,0.87,0.4];

x11 = [60 110 110 60];
y11 = [-10 -10 10 10];
handle_attack = patch(x11,y11,'red','FaceAlpha',0.2,'EdgeColor','none');
hold on
x11 = [200 220 220 200];
y11 = [-10 -10 10 10];
handle_attack = patch(x11,y11,'red','FaceAlpha',0.2,'EdgeColor','none');
hold on
x11 = [240 260 260 240];
y11 = [-10 -10 10 10];
handle_attack = patch(x11,y11,'red','FaceAlpha',0.2,'EdgeColor','none');
ylabel('$x_1$','interpreter','latex','FontSize',24)
hold on
x11 = [401 420 420 401];
y11 = [-10 -10 10 10];
handle_attack = patch(x11,y11,'red','FaceAlpha',0.2,'EdgeColor','none');
hold on
handle_new = plot(x(1,:),'color',[0    0.4471    0.7412],'LineWidth',2)
hold on
handle_old = plot(x_data(1,:),'color','green','LineWidth',2,'LineStyle',':')
hold on
handle_w = plot(x_w(1,:),'color',[0.7176    0.2745    1.0000],'LineWidth',3.5,'LineStyle',':');
hold on
handle_ref = plot(r(1,:),'r--','LineWidth',1.5)
box on
xticks([]);
xlim([0 sim_time])

f2 = subplot(2,1,2)
f2.InnerPosition = [0.096,0.15,0.87,0.4];
x11 = [60 110 110 60];
y11 = [-30 -30 30 30];
handle_attack = patch(x11,y11,'red','FaceAlpha',0.2,'EdgeColor','none');
hold on
x11 = [200 220 220 200];
y11 = [-30 -30 30 30];
handle_attack = patch(x11,y11,'red','FaceAlpha',0.2,'EdgeColor','none');
hold on
x11 = [240 260 260 240];
y11 = [-30 -30 30 30];
handle_attack = patch(x11,y11,'red','FaceAlpha',0.2,'EdgeColor','none');

hold on
x11 = [401 420 420 401];
y11 = [-30 -30 30 30];
handle_attack = patch(x11,y11,'red','FaceAlpha',0.2,'EdgeColor','none');

hold on
handle_new = plot(x(2,:),'color',[0    0.4471    0.7412],'LineWidth',2)
hold on
handle_old = plot(x_data(2,:),'g:','LineWidth',2)
hold on
handle_w = plot(x_w(2,:),'color',[0.7176    0.2745    1.0000],'LineWidth',3.5,'LineStyle',':');
hold on
handle_ref = plot(r(2,:),'r--','LineWidth',1.5);
xlim([0 sim_time])

ylim([-30 30])
xlabel('$k$','interpreter','latex','FontSize',24);
ylabel('$x_2$','interpreter','latex','FontSize',24)
box on
legend([handle_new,handle_w,handle_old,handle_ref,handle_attack],...
    'Proposed approach',...
    'No Attack',...
    'Approach in [12]',...
    '$r_k$',...
    'Attack period','Interpreter','Latex',...
    'Location','best','FontSize',9)

print -depsc -tiff -r300 -painters state_evolutions.eps

%% Computing tracking error


for i=1:sim_time
    sum_proposed(i) = abs(r(i) - x(i));
    sum_previous_app(i) = abs(r(i) - x_data(i));
    sum_without_attack(i) = abs(r(i) - x_w(i));
end

fprintf('=========================================================================================')
fprintf('\n')
fprintf('tracking error - proposed method: %d\n',mean(sum_proposed))
fprintf('\n')
fprintf('tracking error - proposed method in [12]: %d\n',mean(sum_previous_app))
fprintf('\n')
fprintf('tracking error - absence of attack: %d\n',mean(sum_without_attack))

%% Visualization 
fprintf('=========================================================================================')
fprintf('\n')
fprintf('visualization!')
run('visualizations.m')

