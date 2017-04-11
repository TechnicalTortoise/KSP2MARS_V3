function plotvelocity(time, final_xvel, final_yvel)
%CREATEFIGURE(TIME1, FINAL_XVEL1, FINAL_YVEL1)
%  TIME1:  vector of x data
%  FINAL_XVEL1:  vector of y data
%  FINAL_YVEL1:  vector of y data

%  Auto-generated by MATLAB on 05-Apr-2017 04:26:23

% Create figure
figure1 = figure('Color',[1 1 1]);

% Create axes
axes1 = axes('Parent',figure1,...
    'Position',[0.128154981549816 0.526040044634934 0.775 0.341162790697675]);
hold(axes1,'on');

% Create plot
plot(time,final_xvel,'Parent',axes1,'LineWidth',2,'LineStyle','-.',...
    'Color',[0.850980401039124 0.325490206480026 0.0980392172932625]);

% Create title
title('Speed vs Time','FontSize',24);

% Create ylabel
ylabel('Horizontal velocity (m/s)','FontWeight','bold','FontSize',13.2);

box(axes1,'on');
% Set the remaining axes properties
set(axes1,'Color',[0.972549021244049 0.972549021244049 0.972549021244049],...
    'FontSize',12,'XGrid','on','YGrid','on');
% Create axes
axes2 = axes('Parent',figure1,...
    'Position',[0.128154981549816 0.140534351145038 0.775 0.341162790697675]);
hold(axes2,'on');

% Create plot
plot(time,final_yvel,'Parent',axes2,'DisplayName','final_x','LineWidth',2,...
    'LineStyle','-.',...
    'Color',[0 0.447058826684952 0.74117648601532]);

% Create xlabel
xlabel('Time (s)','FontWeight','bold','FontSize',13.2);

% Create ylabel
ylabel('Vertical velocity (m/s)','FontWeight','bold','FontSize',13.2);

box(axes2,'on');
% Set the remaining axes properties
set(axes2,'Color',[0.972549021244049 0.972549021244049 0.972549021244049],...
    'FontSize',12,'XGrid','on','YGrid','on');
% % Create line
% annotation(figure1,'line',[0.128125 0.903125],...
%     [0.1886586695747 0.1886586695747],'LineWidth',1.5,'LineStyle',':');

