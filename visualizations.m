%% Trajectory Visualization
% clc
% close all
f = figure;
% Set axis limits
axis([-10 10 -30 30]);

% Set aspect ratio
pbaspect([1 3 1]);

% Set figure size
set(gcf, 'Position', [200, 100, 550, 600]);

% Set inner position of the axes
ax = gca; % Get current axes
ax.InnerPosition = [0.11, 0.1, 0.87, 0.898]; % [left, bottom, width, height]


for i=1:5
    plot(V{i},'Alpha',0,'color','white','EdgeColor','r','LineWidth',2)
    hold on
end
hold on
box on
xlabel('$x_1$','interpreter','latex','FontSize',24)
ylabel('$x_2$','interpreter','latex','FontSize',24)
p1 = [4;-6;0;6;-4];
p2 = [15;15;0;-20;-20];
for i=1:set_num
    plot(p1(i),p2(i),'*','Color',[1.0000    0.5529    0.3098],'MarkerSize',10)
    hold on
end
hold on

xlim([-10.5 10.5]);
ylim([-20.5 30.5]);

text(-9,27,'$\mathcal{X}_{\eta}$','FontSize',18,'interpreter','latex')
hold on
annotation('textbox',...
    [0.6842435897436 0.607 0.0637564102564 0.0574972673196976],'String','$\mathcal{V}_1$',...
    'Interpreter','latex',...
    'FontSize',18,...
    'FitBoxToText','off','EdgeColor',[0.501960784313725 0.501960784313725 0.501960784313725]);
annotation('textbox',...
    [0.172464452214453 0.597666666666667 0.0649900932400928 0.0596951046764139],'String','$\mathcal{V}_2$',...
    'Interpreter','latex',...
    'FontSize',18,...
    'FitBoxToText','off','EdgeColor',[0.501960784313725 0.501960784313725 0.501960784313725]);
annotation('textbox',...
    [0.186 0.431333333333334 0.066 0.0557217184936825],'String','$\mathcal{V}_3$',...
    'Interpreter','latex',...
    'FontSize',18,...
    'FitBoxToText','off','EdgeColor',[0.501960784313725 0.501960784313725 0.501960784313725]);
annotation('textbox',...
    [0.736363636363636 0.211 0.0628727272727264 0.0568964048160321],'String','$\mathcal{V}_4$',...
    'Interpreter','latex',...
    'FontSize',18,...
    'FitBoxToText','off','EdgeColor',[0.501960784313725 0.501960784313725 0.501960784313725]);
annotation('textbox',...
    [0.357842424242425 0.212 0.063975757575757 0.0547540119558812],'String','$\mathcal{V}_5$',...
    'Interpreter','latex',...
    'FontSize',18,...
    'FitBoxToText','off','EdgeColor',[0.501960784313725 0.501960784313725 0.501960784313725]);

hold on
text(4,17,'$x^1_e$','FontSize',18,'interpreter','latex')
hold on 
text(-7.5,15.61,'$x^2_e$','FontSize',18,'interpreter','latex')
hold on
text(0.48,-0.10,'$x^3_e$','FontSize',18,'interpreter','latex')
hold on 
text(6,-18.5,'$x^4_e$','FontSize',18,'interpreter','latex')
hold on 
text(-4,-18.5,'$x^5_e$','FontSize',18,'interpreter','latex')

plot(Td_f,[1 2],'LineWidth',2)
hold on 
for k=1:sim_time
    if k>=60 & k <=107
        plot(S{k},[1 2],'LineWidth',0.001,'EdgeColor','black','FaceColor',...
            [0.9804    0.4118    0.4118]);
    elseif (k>=200  & k<=221) | (k>=240  & k<=261)
        plot(S{k},[1 2],'LineWidth',0.001,'EdgeColor',...
            'black','FaceColor',[0.9098    0.8196    0.1255]);
    elseif k>=401  & k<=421
        plot(S{k},[1 2],'LineWidth',0.001,'EdgeColor',...
           'black','FaceColor',[0.0941    0.6902    0.0157]);
    end
  
end
hold on 
handle_tracking = plot(x_w(1,1:sim_time),x_w(2,1:sim_time),'color',[0.7176    0.2745    1],'LineWidth',3,...
    'LineStyle',':');

hold on
handle_tracking = plot(x(1,1:sim_time),x(2,1:sim_time),'color',[0    0.4471    0.7412],'LineWidth',1,...
    'LineStyle','-'); 

hold on 
annotation('textarrow',[0.378153846153846 0.331384615384615],...
    [0.416307692307692 0.465538461538462],...
    'String',{'$J_{402}=15.52$'},'interpreter','latex','FontSize',15);

hold on 
annotation('textarrow',[0.439692307692308 0.488923076923077],...
    [0.912307692307692 0.888923076923077],...
    'String',{'$J_{413}=3.31$'},...
    'interpreter','latex','FontSize',15);
hold on 

annotation('textarrow',[0.3748 0.461846153846154],...
    [0.8428 0.853230769230769],'String',{'$J_{411}=8.23$'},'interpreter',...
    'latex','FontSize',15);
hold on 
annotation('textarrow',[0.625818181818182 0.615636363636364],...
    [0.872333333333334 0.925666666666667],'String',{'$J_{416}=1.44$'},'interpreter',...
    'latex','FontSize',15);

grid on
annotation('line',[0.830909090909091 0.91476923076923],...
    [0.855 0.829846153846154],'LineWidth',0.5);

annotation('ellipse',...
    [0.908615384615385 0.789307692307692 0.054153846153846 0.0529230769230769],...
    'LineWidth',0.5);

r1 = [-1;9;-6;-7;3;4];
r2 = [2;20;-7;-15;27;25];
for i=1:6
    plot(r1(i),r2(i),'b*','MarkerSize',10)
    hold on
end
annotation('textbox',...
    [0.786215384615385 0.855882674335012 0.139107692307692 0.0365471078555099],...
    'String','Zoom in',...
    'FitBoxToText','off',...
    'EdgeColor','None');

hold on 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Box %%%%%%%%%%%%%%%%%%%%%%
ax=axes;
set(ax,'units','normalized','position',[0.725118881118881 0.855692307692308 0.209230769230769 0.139467464588959])
box(ax,'on')
xticks([])
yticks([])
hold on 
plot(S{107},[1 2],'LineWidth',0.001,'EdgeColor','None','FaceColor',...
            [0.9804    0.4118    0.4118]); 
hold on 
r1 = [-1;9;-6;-7;3;4];
r2 = [2;20;-7;-15;27;25];
for i=1:6
    plot(r1(i),r2(i),'b*','MarkerSize',10)
    hold on
end
hold on 
text(8.8,21.2,'$\hat\mathcal{R}_{107}$','FontSize',12.5,'interpreter','latex')
hold on 
plot(Td_f,[1 2],'LineWidth',2)
hold on 

xlim([8.5 10])
ylim([17 22])


print -depsc -tiff -r600 -painters trajectories.eps



