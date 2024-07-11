clc
close all

X = zonotope(interval([-2.5;-10],[2.5;10])); % constraints on states
U = zonotope(0,5); % constraint on input
W = zonotope(zeros(dim_x,1),0.001*eye(dim_x));

Td = load('Td.mat').Td;   % ROSC sets
% V_AB = load('V_AB.mat').V_AB;  % set of vertices \mathcal{V}_{AB} as a matrix zonotope
% AB = load('AB.mat').AB;   % set of system matrices \mathcal{M}_{AB} as a matrix zonotope
Td_aug = load('Td_aug.mat').Td_aug;  % set of augmented ROSC sets

figure;
% set(gca, 'Position',[0.099 0.12 0.897 0.87])
index = 1;
projectedDims = {[1 2]};

for plotRun=1: length(projectedDims)
    index = index + 1;
    
    % plot RCI set
%     handleX0 = plot(Td{1},[1 2],'r--','LineWidth',0.75);
    
    % plot model-based ROSC sets
    hold on
    for iSet=2:80
        handleModel=  plot(Td{iSet},[1 2],'b-','LineWidth',0.75);
    end
    hold on
    
    %%%%
    xlabel('$x_1$','interpreter','latex','FontSize',20)
    ylabel('$x_2$','interpreter','latex','FontSize',20)
    warOrig = warning; warning('off','all');
    exportgraphics(gcf,'STMPC_controller.eps','BackgroundColor','none','ContentType','vector')
end
%
grid off

hold on
x_curr = [0; -0.1]; % initial state
x1 = x_curr;

index_data = [];  % set membership for data-driven ST-MPC
index_data(1) = set_index(x1,Td)

u1 = [];
i = 0;

% plot initial state
handleInitial_state = plot(x_curr(1),x_curr(2),'go','MarkerFaceColor', 'g',...
    'MarkerEdgeColor','k','MarkerSize',4)
hold on
% pause(7)
data_traj_x1(1) = x1(1);
data_traj_x2(1) = x1(2);
sim_time = 100;

while i < sim_time
    W_k = randPoint(W,1,'standard');
    % plot initial state
    hold on
   
    u1 = one_step_ctrl(1, x1, Td_aug, index_data(i+1));
    %
   
    x1 = sys_d.A*x1 + sys_d.B*u1 + W_k;
    u_data(i+1) = u1;
    hand_data_traj = plot(x1(1), x1(2), 'ko', 'MarkerFaceColor',...
        'k', 'MarkerEdgeColor', 'k','MarkerSize', 3)
    hold on
    data_traj_x1(i+2) = x1(1);
    data_traj_x2(i+2) = x1(2);
       
    % plot state evolution
    hold on
    index_data(i+2) = set_index(x1,Td)
    pause(0.1)
    i = i + 1;
 
end

