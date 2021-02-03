
hold off

%log3 = readtable('log_3_Decoded.csv','VariableNamingRule', 'preserve');
log3 = readtable('log_4_Decoded.csv','VariableNamingRule', 'preserve');

temp3 = log3.Temperature;
press3 = log3.Pressure/1000.000;

%date times 
log3.Date.Format = 'dd/MM/yyyy HH:mm:ss';
datearr = log3.Date + log3.Time;

x_vals = datenum(datearr);
% first axis for time in hours
ax1 = gca;
datetick(ax1,'x','HH:MM');
ax1_pos = ax1.Position; % position of first axes
% Second Axis for date
ax2 = axes('Position',ax1_pos,...
    'XAxisLocation','bottom',...
    'Color','none');
ax2.XTick = x_vals;
ax2.XRuler.TickLabelGapOffset = +16;

figure(1);
plot(x_vals,temp3);
[t,s] = title("Ambient Temperature measured by SHARC Buoy from 1 Feb 2021 to 3 Feb 2021");
t.FontSize = 18;
grid(ax1,'on');
grid(ax2,'minor');
datetick(ax2,'x', 'dd mmm')
set(ax1,'YTickLabel',[]);
ylabel("Temperature (°C)")
xlim([x_vals(1), x_vals(end)])

figure(2)
% first axis for time in hours
ax1 = gca;
datetick(ax1,'x','HH:MM');
ax1_pos = ax1.Position; % position of first axes
% Second Axis for date
ax2 = axes('Position',ax1_pos,...
    'XAxisLocation','bottom',...
    'Color','none');
ax2.XTick = x_vals;
ax2.XRuler.TickLabelGapOffset = +16;
plot(x_vals,press3)
[t,s] = title("Atmospheric Pressure measured by SHARC Buoy from  1 Feb 2021 to 3 Feb 2021");
t.FontSize = 18;
grid(ax1,'on');
grid(ax2,'minor');
datetick(ax2,'x', 'dd mmm')
set(ax1,'YTickLabel',[]);
ylabel("Pressure (KPa)")
ylim([min(press3),max(press3)])
xlim([x_vals(1), x_vals(end)])