
hold off

log3 = readtable('PolarStern_Buoy1_Decoded.csv','VariableNamingRule', 'preserve');

temp3 = log3.Temperature;
press3 = log3.Pressure/1000.000;

%date times 
log3.Date.Format = 'dd/MM/yyyy HH:mm:ss';
datearr = log3.Date + log3.Time;
date_min = datestr(min(log3.Date));
date_max = datestr(max(log3.Date));
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
figure('units','normalized','outerposition',[0 0 1 1])
plot(x_vals,temp3);
[t,s] = title("Ambient Temperature measured by SHARC Buoy from " +date_min+" to "+date_max);
t.FontSize = 18;
grid(ax1,'on');
grid(ax2,'minor');
datetick(ax2,'x', 'dd mmm')
set(ax1,'YTickLabel',[]);
ylabel("Temperature (°C)")
xlim([x_vals(1), x_vals(end)])
saveas(gcf,"temp_"+date_min+"_"+date_max+".png")
figure(2)
% first axis for time in hours
figure('units','normalized','outerposition',[0 0 1 1])
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
[t,s] = title("Atmospheric Pressure measured by SHARC Buoy from " +date_min+" to "+date_max);
t.FontSize = 18;
grid(ax1,'on');
grid(ax2,'minor');
datetick(ax2,'x', 'dd mmm')
set(ax1,'YTickLabel',[]);
ylabel("Pressure (KPa)")
ylim([min(press3),max(press3)])
xlim([x_vals(1), x_vals(end)])
saveas(gcf,"press_"+date_min+"_"+date_max+".png")
% battery voltage

V_bat = table2array(log3(:,15))+ table2array(log3(:,14))/1000.00 ;
Current = table2array(log3(:,16));
Power =  table2array(log3(:,17));

figure(3)
figure('units','normalized','outerposition',[0 0 1 1])
ax1 = gca;
datetick(ax1,'x','HH:MM');
ax1_pos = ax1.Position; % position of first axes
% Second Axis for date
ax2 = axes('Position',ax1_pos,...
    'XAxisLocation','bottom',...
    'Color','none');
ax2.XTick = x_vals;
ax2.XRuler.TickLabelGapOffset = +16;
plot(x_vals,V_bat);
[t,s] = title("Battery Voltage from " +date_min+" to "+date_max);
t.FontSize = 18;
grid(ax1,'on');
grid(ax2,'minor');
datetick(ax2,'x', 'dd mmm')
set(ax1,'YTickLabel',[]);
ylabel("Voltage (V)")
ylim([min(V_bat),max(V_bat)])
xlim([x_vals(1), x_vals(end)])
saveas(gcf,"batt_"+date_min+"_"+date_max+".png")

figure(4)
figure('units','normalized','outerposition',[0 0 1 1])
ax1 = gca;
datetick(ax1,'x','HH:MM');
ax1_pos = ax1.Position; % position of first axes
% Second Axis for date
ax2 = axes('Position',ax1_pos,...
    'XAxisLocation','bottom',...
    'Color','none');
ax2.XTick = x_vals;
ax2.XRuler.TickLabelGapOffset = +16;
plot(x_vals,Current);
[t,s] = title("Current from " +date_min+" to "+date_max);
t.FontSize = 18;
grid(ax1,'on');
grid(ax2,'minor');
datetick(ax2,'x', 'dd mmm')
set(ax1,'YTickLabel',[]);
ylabel("Current (mA)")
ylim([min(Current),max(Current)])
xlim([x_vals(1), x_vals(end)])
saveas(gcf,"curr_"+date_min+"_"+date_max+".png")

figure(5)
figure('units','normalized','outerposition',[0 0 1 1])
ax1 = gca;
datetick(ax1,'x','HH:MM');
ax1_pos = ax1.Position; % position of first axes
% Second Axis for date
ax2 = axes('Position',ax1_pos,...
    'XAxisLocation','bottom',...
    'Color','none');
ax2.XTick = x_vals;
ax2.XRuler.TickLabelGapOffset = +16;
plot(x_vals,Power);
[t,s] = title("Power consumption from " +date_min+" to "+date_max);
t.FontSize = 18;
grid(ax1,'on');
grid(ax2,'minor');
datetick(ax2,'x', 'dd mmm')
set(ax1,'YTickLabel',[]);
ylabel("Power (mW)")
ylim([min(Power),max(Power)])
xlim([x_vals(1), x_vals(end)])
saveas(gcf,"pwr_"+date_min+"_"+date_max+".png")