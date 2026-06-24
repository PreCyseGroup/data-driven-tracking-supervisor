% Author:       Mehran Attar - Montreal, Canada
% Revised:      Data-driven vs model-based simulation + conservatism analysis
% Modified:     Added conditional real-time plotting of one-step reachable sets
% Purpose:
% This script runs:
%   1) Data-driven architecture using M_AB.
%   2) Model-based counterpart using true A,B for reachable-set prediction
%      and safety verification.
%
% It computes and prints:
%   - Average volume of model-based one-step reachable sets.
%   - Average volume of data-driven one-step reachable sets.
%   - Average percentage conservatism.
%   - Number and rate of false-positive SV/EC activations.
%   - Tracking errors using the same formula as the original code.
%
% This version plots one-step forward reachable sets only if:
%   1) The system is under attack.
%   2) The emergency controller is not active.

clc
clear all
close all

w = warning('off','all');
try
    rmpath('folderthatisnotonpath')
catch
end
warning(w)

%% ================================================================
% System matrices
% ================================================================

A = [0.9719 0.0013;
     0.0340 0.8628];

B = [-0.0839 0.0232;
      0.0761 0.4144];

C = [1 0;
     0 1];

D = zeros(2,2);

dim_x = size(A,1);
dim_u = size(B,2);

sys = ss(A,B,C,D);

%% ================================================================
% Constraints and disturbance
% ================================================================

X = zonotope(interval([-10;-30],[10;30]));
U = zonotope(interval([-2;-10],[2;10]));
W = zonotope(zeros(2,1),0.001*eye(2));

%% ================================================================
% Tracking controller
% ================================================================

Q = eye(dim_x);
R = eye(dim_u);
K = dlqr(A,B,Q,R);

%% ================================================================
% Load ROSC sets, Voronoi regions, tracking-controller domain
% ================================================================

Td1     = load('Td1').Td1;
Td1_aug = load('Td1_aug.mat').Td1_aug;
V       = load('V').V;
Td_f    = load('Td_f').Td_f;

%% ================================================================
% Compute data-driven model set M_AB
% ================================================================

initpoints = 2;
steps = 2;

[AB,X_0T,U_full] = compute_AB(sys,X,U,W,initpoints,steps);

%% ================================================================
% Simulation settings
% ================================================================

sim_time = 600;
set_num = 5;
D_delay = 5;

x0 = [0.01;-0.01];

real_time_plot = true;
pause_time = 0.005;

% Reachable-set visualization options
plot_reachable_sets = true;
plot_ts_prediction_sets = true;
reachable_set_plot_every = 1;

% Important:
% If true, reachable sets are plotted only during attack intervals.
plot_sets_only_during_attack = true;

% Important:
% If true, reachable sets are not plotted when EC is active.
do_not_plot_sets_during_emergency = true;

%% ================================================================
% Reference signal
% ================================================================

r = zeros(dim_x,sim_time);

for i = 1:sim_time
    if i < 60
        r(:,i) = [-1;2];
    elseif i >= 60 && i < 200
        r(:,i) = [9;20];
    elseif i >= 200 && i < 300
        r(:,i) = [-6;-7];
    elseif i >= 300 && i < 400
        r(:,i) = [-7;-15];
    elseif i >= 400 && i < 500
        r(:,i) = [3;27];
    else
        r(:,i) = [4;25];
    end
end

%% ================================================================
% Measurement-channel FDI attack
% ================================================================

y_a = zeros(dim_x,sim_time+1);
attack = false(1,sim_time+1);

for i = 1:sim_time+1
    if i >= 60 && i <= 125
        y_a(:,i) = 0.01*[(i-59);(i-59)];
        attack(i) = true;
    elseif i >= 200 && i <= 220
        y_a(:,i) = 0.08*[(i-199);(i-199)];
        attack(i) = true;
    elseif i >= 240 && i <= 260
        y_a(:,i) = [0.1*(i-239);0.1*(i-239)];
        attack(i) = true;
    elseif i > 400 && i <= 420
        y_a(:,i) = [0.1*(i-399);0.1*(i-399)];
        attack(i) = true;
    else
        y_a(:,i) = [0;0];
        attack(i) = false;
    end
end

%% ================================================================
% Actuation-channel FDI attack
% ================================================================

u_a = zeros(dim_u,sim_time);

for i = 1:sim_time
    if i >= 150 && i < 160
        u_a(:,i) = [0;0];
    else
        u_a(:,i) = [0;0];
    end
end

%% ================================================================
% Equilibrium feedforward input for reference tracking
% ================================================================

ss_input = zeros(dim_u,sim_time);

for i = 1:sim_time
    ss_input(:,i) = pinv(C * inv(eye(size(A)) - A + B*K) * B) * r(:,i);
end

%% ================================================================
% Performance index matrix I
% ================================================================

p1 = [4;-6;0;6;-4];
p2 = [15;15;0;-20;-20];

I = cell(set_num,set_num);

for i = 1:set_num
    for j = 1:set_num
        I{i,j} = sqrt((p1(i)-p1(j))^2 + (p2(i)-p2(j))^2);
    end
end

%% ================================================================
% Real-time state-space visualization
% ================================================================

if real_time_plot
    f_state = figure;
    f_state.Position = [700 70 800 700];
    ax = axes;
    hold(ax,'on')
    box(ax,'on')

    plot(x0(1),x0(2),'*','MarkerSize',5,'MarkerEdgeColor','k')
    hold on

    for i = 1:set_num
        plot(V{i},'Alpha',0.01,'color','white','EdgeColor','r','LineWidth',2)
        hold on
    end

    plot(Td_f)
    hold on

    for i = 1:set_num
        plot(p1(i),p2(i),'r*','MarkerSize',6)
        hold on
    end

    ref_points = unique(r','rows')';
    for i = 1:size(ref_points,2)
        plot(ref_points(1,i),ref_points(2,i),'b*','MarkerSize',6)
        hold on
    end

    xlabel('$x_1$','interpreter','latex','FontSize',24)
    ylabel('$x_2$','interpreter','latex','FontSize',24)
    xlim([-11 11])
    ylim([-31 31])
    title('Real-time state-space trajectories and attack-time reachable sets','interpreter','latex')

    h_DD = plot(NaN,NaN,'ko','MarkerSize',3,'MarkerFaceColor','k');
    h_MB = plot(NaN,NaN,'mo','MarkerSize',3,'MarkerFaceColor','m');

    h_DD_reach = plot(NaN,NaN,'-','Color',[0 0.4470 0.7410],'LineWidth',1);
    h_MB_reach = plot(NaN,NaN,'--','Color',[0.8500 0.3250 0.0980],'LineWidth',1);

%     legend([h_DD,h_MB,h_DD_reach,h_MB_reach], ...
%         'Data-driven trajectory', ...
%         'Model-based trajectory', ...
%         'Data-driven one-step reachable set', ...
%         'Model-based one-step reachable set', ...
%         'Interpreter','latex', ...
%         'Location','best')
end

%% ================================================================
% Parameter structure
% ================================================================

params = struct();
params.A = A;
params.B = B;
params.K = K;
params.W = W;
params.U = U;
params.Td_f = Td_f;
params.Td1 = Td1;
params.Td1_aug = Td1_aug;
params.V = V;
params.AB = AB;
params.r = r;
params.ss_input = ss_input;
params.y_a = y_a;
params.attack = attack;
params.u_a = u_a;
params.I = I;
params.D_delay = D_delay;
params.sim_time = sim_time;
params.x0 = x0;
params.dim_x = dim_x;
params.dim_u = dim_u;
params.real_time_plot = real_time_plot;
params.pause_time = pause_time;

params.plot_reachable_sets = plot_reachable_sets;
params.plot_ts_prediction_sets = plot_ts_prediction_sets;
params.reachable_set_plot_every = reachable_set_plot_every;
params.plot_sets_only_during_attack = plot_sets_only_during_attack;
params.do_not_plot_sets_during_emergency = do_not_plot_sets_during_emergency;

if real_time_plot
    params.ax = ax;
else
    params.ax = [];
end

%% ================================================================
% Run data-driven simulation
% ================================================================

fprintf('=========================================================================================\n')
fprintf('Running DATA-DRIVEN architecture\n')
fprintf('=========================================================================================\n')

results_DD = run_architecture_simulation('data_driven',params);

%% ================================================================
% Run model-based simulation
% ================================================================

fprintf('=========================================================================================\n')
fprintf('Running MODEL-BASED counterpart\n')
fprintf('=========================================================================================\n')

results_MB = run_architecture_simulation('model_based',params);

%% ================================================================
% Conservatism analysis along data-driven simulation
% ================================================================

valid_idx = isfinite(results_DD.vol_dd) & ...
            isfinite(results_DD.vol_mb) & ...
            results_DD.vol_mb > 1e-12;

avg_vol_model_based = mean(results_DD.vol_mb(valid_idx));
avg_vol_data_driven = mean(results_DD.vol_dd(valid_idx));

avg_percent_conservatism = mean(results_DD.vol_gap_percent(valid_idx));

false_positive_count = sum(results_DD.false_positive_sv(valid_idx));
false_positive_rate = 100 * false_positive_count / sum(valid_idx);

fprintf('\n')
fprintf('=========================================================================================\n')
fprintf('Reachable-set conservatism analysis along the data-driven simulation\n')
fprintf('=========================================================================================\n')
fprintf('Average volume - model-based reachable set:      %.6e\n',avg_vol_model_based)
fprintf('Average volume - data-driven reachable set:      %.6e\n',avg_vol_data_driven)
fprintf('Average percentage conservatism:                 %.2f %%\n',avg_percent_conservatism)
fprintf('False-positive SV/EC activations:                %d\n',false_positive_count)
fprintf('False-positive SV/EC activation rate:            %.2f %%\n',false_positive_rate)

conservatism_results = table( ...
    avg_vol_model_based, ...
    avg_vol_data_driven, ...
    avg_percent_conservatism, ...
    false_positive_count, ...
    false_positive_rate);

disp(conservatism_results)

%% ================================================================
% Tracking error using the same formula as the original code
% ================================================================

sum_DD = zeros(1,sim_time);
sum_MB = zeros(1,sim_time);

for i = 1:sim_time
    sum_DD(i) = abs(r(i) - results_DD.x(i));
    sum_MB(i) = abs(r(i) - results_MB.x(i));
end

tracking_error_DD = mean(sum_DD);
tracking_error_MB = mean(sum_MB);

fprintf('\n')
fprintf('=========================================================================================\n')
fprintf('Tracking-error comparison using the original code formula\n')
fprintf('=========================================================================================\n')
fprintf('Mean tracking error - data-driven architecture:  %.6f\n',tracking_error_DD)
fprintf('Mean tracking error - model-based counterpart:   %.6f\n',tracking_error_MB)

%% ================================================================
% Save results
% ================================================================

save('DD_vs_MB_conservatism_results.mat', ...
    'results_DD', ...
    'results_MB', ...
    'conservatism_results', ...
    'avg_vol_model_based', ...
    'avg_vol_data_driven', ...
    'avg_percent_conservatism', ...
    'false_positive_count', ...
    'false_positive_rate', ...
    'tracking_error_DD', ...
    'tracking_error_MB');

fprintf('\nDone.\n')

%% ================================================================
% Local functions
% ================================================================

function results = run_architecture_simulation(mode,params)

    A = params.A;
    B = params.B;
    K = params.K;
    W = params.W;
    U = params.U;
    Td_f = params.Td_f;
    Td1 = params.Td1;
    Td1_aug = params.Td1_aug;
    V = params.V;
    AB = params.AB;
    r = params.r;
    ss_input = params.ss_input;
    y_a = params.y_a;
    attack = params.attack;
    u_a = params.u_a;
    I = params.I;
    D_delay = params.D_delay;
    sim_time = params.sim_time;
    x0 = params.x0;
    dim_x = params.dim_x;
    dim_u = params.dim_u;
    real_time_plot = params.real_time_plot;
    pause_time = params.pause_time;
    ax = params.ax;

    plot_reachable_sets = params.plot_reachable_sets;
    plot_ts_prediction_sets = params.plot_ts_prediction_sets;
    reachable_set_plot_every = params.reachable_set_plot_every;
    plot_sets_only_during_attack = params.plot_sets_only_during_attack;
    do_not_plot_sets_during_emergency = params.do_not_plot_sets_during_emergency;

    if strcmp(mode,'data_driven')
        state_color = [0 0 0];
        reach_color = [0 0.4470 0.7410];
        ts_color = [0 0.4470 0.7410];
        marker_style = 'ko';
        marker_face = 'k';
        reach_line_style = '-';
    else
        state_color = [1 0 1];
        reach_color = [0.8500 0.3250 0.0980];
        ts_color = [0.4940 0.1840 0.5560];
        marker_style = 'mo';
        marker_face = 'm';
        reach_line_style = '--';
    end

    x = zeros(dim_x,sim_time+1);
    x_prime = zeros(dim_x,sim_time+1);
    alarm = zeros(1,sim_time+1);

    x(:,1) = x0;
    x_prime(:,1) = x0;

    u_nom = zeros(dim_u,sim_time);
    u_open_loop = zeros(dim_u,sim_time);
    u_applied = zeros(dim_u,sim_time);

    emergency = zeros(1,sim_time);
    safety = zeros(1,sim_time);

    S = cell(1,sim_time+1);
    S_alarm = cell(1,sim_time);
    J = cell(1,sim_time+1);

    vol_dd = nan(1,sim_time);
    vol_mb = nan(1,sim_time);
    vol_gap_percent = nan(1,sim_time);

    safe_dd = nan(1,sim_time);
    safe_mb = nan(1,sim_time);
    false_positive_sv = false(1,sim_time);

    flag = 0;
    init_ts = 0;
    ts_flag = 0;

    for k = 1:sim_time

        fprintf('[%s] time = %d\n',mode,k)

        index_r = partition_index(V,r(:,k));

        % Nominal tracking controller
        u_nom(:,k) = TC(x(:,k),ss_input(:,k),K);

        if alarm(k) == 0
            ts_flag = 0;
        end

        % Initialize Tracking Supervisor
        if alarm(k) == 1 && flag == 0
            if init_ts == 0

                k0 = max(1,k-D_delay);

                if strcmp(mode,'data_driven')
                    S{k} = TS_Init(x(:,k0),u_nom(:,k0),AB,W);
                else
                    S{k} = model_based_reach(A,B,x(:,k0),u_nom(:,k0),W);
                end

                J{k} = performance_cal(V,index_r,S{k},I);

                init_ts = 1;
                ts_flag = 1;
            end
        end

        ts_prediction_set_available = false;

        % Run Tracking Supervisor
        if init_ts == 1 && ts_flag == 1

            x_hat = randPoint(S{k});
            u_open_loop(:,k) = TC(x_hat,ss_input(:,k),K);

            if strcmp(mode,'data_driven')
                S{k+1} = TS(S{k},u_open_loop(:,k),AB,W);
            else
                S{k+1} = model_based_reach_from_set(A,B,S{k},u_open_loop(:,k),W);
            end

            try
                S{k+1} = reduce(S{k+1},'girard',20);
            catch
            end

            ts_prediction_set_available = true;

            J{k+1} = performance_cal(V,index_r,S{k+1},I);

            if J{k+1} > J{k}
                flag = 1;
            end

            if contains_set_local(Td_f,S{k}) == 0
                flag = 1;
                init_ts = 0;
                ts_flag = 0;
            else
                ts_flag = 1;
            end
        end

        % Candidate input for safety verification
        if ts_flag == 1 && flag == 0
            u_candidate = u_open_loop(:,k);
        else
            u_candidate = u_nom(:,k);
        end

        % Reachable set used by the current architecture
        if strcmp(mode,'data_driven')
            S_alarm{k} = data_driven_reach(AB,x(:,k),u_candidate,W,dim_x,dim_u);
        else
            S_alarm{k} = model_based_reach(A,B,x(:,k),u_candidate,W);
        end

        % Conservatism analysis for the same x_k and u_candidate
        S_dd_tmp = data_driven_reach(AB,x(:,k),u_candidate,W,dim_x,dim_u);
        S_mb_tmp = model_based_reach(A,B,x(:,k),u_candidate,W);

        vol_dd(k) = zonotope_area_2d_local(S_dd_tmp);
        vol_mb(k) = zonotope_area_2d_local(S_mb_tmp);

        if vol_mb(k) > 1e-12
            vol_gap_percent(k) = 100*(vol_dd(k)-vol_mb(k))/vol_mb(k);
        end

        safe_dd(k) = contains_set_local(Td_f,S_dd_tmp);
        safe_mb(k) = contains_set_local(Td_f,S_mb_tmp);

        false_positive_sv(k) = (safe_mb(k) == 1) && (safe_dd(k) == 0);

        % Voronoi index
        v = partition_index(V,x(:,k));

        if flag == 1 && contains_point_local(Td1{v,1},x(:,k)) == 1
            flag = 0;
        end

        % Safety verification
        if strcmp(mode,'data_driven')
            [~,safety(k)] = data_driven_safety_guard(u_candidate,x(:,k),U,Td_f,AB,W);
        else
            [~,safety(k)] = model_based_safety_guard(A,B,u_candidate,x(:,k),U,Td_f,W);
        end

        if safety(k) == 1
            flag = 1;
        end

        % Switching between EC and tracking controller
        if flag == 1
            index_ec = set_index(x(:,k),Td1,v);
            u_applied(:,k) = one_step_ctrl(2,x(:,k),Td1_aug,index_ec,v);
            emergency(k) = 1;
            fprintf('[%s] Emergency controller is active\n',mode)
        else
            if ts_flag == 1
                u_applied(:,k) = u_open_loop(:,k);
                emergency(k) = 0;
                fprintf('[%s] Open-loop tracking controller is active\n',mode)
            else
                u_applied(:,k) = u_candidate;
                emergency(k) = 0;
                init_ts = 0;
                fprintf('[%s] Tracking controller is active\n',mode)
            end
        end

        % ============================================================
        % Conditional plotting of reachable sets
        % ============================================================
        %
        % The one-step set S_alarm{k} corresponds to the prediction used
        % to check x_prime(:,k+1). Therefore, for plotting during attack
        % periods, we use attack(k) OR attack(k+1).
        %
        % The reachable sets are plotted only if:
        %   1) attack is active,
        %   2) emergency controller is not active,
        %   3) the requested plotting interval is satisfied.
        % ============================================================

        if k < length(attack)
            attack_active_for_plot = attack(k) || attack(k+1);
        else
            attack_active_for_plot = attack(k);
        end

        if plot_sets_only_during_attack == false
            attack_active_for_plot = true;
        end

        emergency_inactive_for_plot = true;

        if do_not_plot_sets_during_emergency == true
            emergency_inactive_for_plot = emergency(k) == 0;
        end

        plot_this_step = real_time_plot && ...
                         attack_active_for_plot && ...
                         emergency_inactive_for_plot && ...
                         mod(k-1,reachable_set_plot_every) == 0;

        if plot_this_step && plot_reachable_sets
            plot_set_local(S_alarm{k},ax,reach_color,0.75,reach_line_style,0.12);
        end

        if plot_this_step && plot_ts_prediction_sets && ts_prediction_set_available
            plot_set_local(S{k+1},ax,ts_color,0.75,'-',0.06);
        end

        % Actuation-channel attack
        u_plant = u_applied(:,k) + u_a(:,k);

        % Plant evolution
        x(:,k+1) = A*x(:,k) + B*u_plant + randPoint(W);

        % Measurement-channel attack
        x_prime(:,k+1) = x(:,k+1) + y_a(:,k+1);

        % Real-time plot of the state trajectory
        if real_time_plot
            plot(ax,x(1,k),x(2,k),marker_style, ...
                'MarkerSize',3, ...
                'MarkerEdgeColor',state_color, ...
                'MarkerFaceColor',marker_face)

            drawnow
            pause(pause_time)
        end

        % Anomaly detector
        alarm(k+1) = detector_data_driven(x_prime(:,k+1),S_alarm{k});
    end

    results = struct();
    results.x = x;
    results.x_prime = x_prime;
    results.u_nom = u_nom;
    results.u_open_loop = u_open_loop;
    results.u_applied = u_applied;
    results.alarm = alarm;
    results.emergency = emergency;
    results.safety = safety;
    results.S_alarm = S_alarm;
    results.vol_dd = vol_dd;
    results.vol_mb = vol_mb;
    results.vol_gap_percent = vol_gap_percent;
    results.safe_dd = safe_dd;
    results.safe_mb = safe_mb;
    results.false_positive_sv = false_positive_sv;
end

function S = data_driven_reach(AB,x,u,W,dim_x,dim_u)

    state = zonotope(x,zeros(dim_x,dim_x));
    input = zonotope(u,zeros(dim_u,dim_u));

    S = AB*(cartProd(state,input)) + W;
end

function S = model_based_reach(A,B,x,u,W)

    dim_x = length(x);
    dim_u = length(u);

    state = zonotope(x,zeros(dim_x,dim_x));
    input = zonotope(u,zeros(dim_u,dim_u));

    S = A*state + B*input + W;
end

function S_next = model_based_reach_from_set(A,B,S,u,W)

    dim_u = length(u);

    input = zonotope(u,zeros(dim_u,dim_u));

    S_next = A*S + B*input + W;
end

function [S_plus,safety_flag] = model_based_safety_guard(A,B,u,x,U,Td_f,W)

    S_plus = model_based_reach(A,B,x,u,W);

    input_is_safe = contains_point_local(U,u);
    successor_is_safe = contains_set_local(Td_f,S_plus);

    if input_is_safe == 1 && successor_is_safe == 1
        safety_flag = 0;
    else
        safety_flag = 1;
    end
end

function val = contains_point_local(SetObj,point)

    try
        val = SetObj.contains(point);
        val = double(logical(val));
        return
    catch
    end

    try
        val = contains(SetObj,point);
        val = double(logical(val));
        return
    catch
    end

    error('Could not evaluate point containment.')
end

function val = contains_set_local(SetObj,Z)

    try
        val = SetObj.contains(Z);
        val = double(logical(val));
        return
    catch
    end

    try
        val = contains(SetObj,Z);
        val = double(logical(val));
        return
    catch
    end

    error('Could not evaluate set containment.')
end

function area_val = zonotope_area_2d_local(Z)

    G = get_generators_local(Z);

    if isempty(G)
        area_val = 0;
        return
    end

    if size(G,1) ~= 2
        area_val = NaN;
        warning('zonotope_area_2d_local only supports 2D zonotopes.')
        return
    end

    m = size(G,2);
    area_val = 0;

    for i = 1:m-1
        for j = i+1:m
            area_val = area_val + abs(det([G(:,i),G(:,j)]));
        end
    end

    area_val = 4*area_val;
end

function G = get_generators_local(Z)

    G = [];

    try
        G = generators(Z);
        return
    catch
    end

    try
        Zmat = Z.Z;
        G = Zmat(:,2:end);
        return
    catch
    end

    try
        G = Z.generators;
        return
    catch
    end

    error('Could not extract generators from zonotope object.')
end

function plot_set_local(SetObj,ax,set_color,line_width,line_style,alpha_val)

    if isempty(ax)
        return
    end

    try
        axes(ax);
    catch
    end

    hold on

    plotted = false;

    try
        plot(SetObj, ...
            'Alpha',alpha_val, ...
            'color',set_color, ...
            'EdgeColor',set_color, ...
            'LineWidth',line_width, ...
            'LineStyle',line_style);
        plotted = true;
    catch
    end

    if ~plotted
        try
            plot(SetObj, ...
                'Alpha',alpha_val, ...
                'Color',set_color, ...
                'EdgeColor',set_color, ...
                'LineWidth',line_width, ...
                'LineStyle',line_style);
            plotted = true;
        catch
        end
    end

    if ~plotted
        try
            h = plot(SetObj);
            try
                set(h,'EdgeColor',set_color);
                set(h,'FaceAlpha',alpha_val);
                set(h,'LineWidth',line_width);
                set(h,'LineStyle',line_style);
            catch
                try
                    set(h,'Color',set_color);
                    set(h,'LineWidth',line_width);
                    set(h,'LineStyle',line_style);
                catch
                end
            end
            plotted = true;
        catch
        end
    end

    if ~plotted
        try
            plot(SetObj);
        catch
            warning('Could not plot the reachable set at this time step.')
        end
    end
end
