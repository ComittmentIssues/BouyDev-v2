close all
% create current plot
%convert Date Time to seconds

T = Time - Time(1);
T_State = State_Time - State_Time(1);
secs = seconds(0:(length(Time)-1));
color1 =[0.8500 0.3250 0.0980];
figure(1)
plot(T,CurrentmA) 
for k= 1:length(T_State)
     xl = xline(T_State(k),'-',State_Name(k),'LineWidth',0.5,'LabelOrientation','horizontal');
     if State_Name(k) == 'Sleep State'
        xl.Color = 'r';

     elseif State_Name(k) == 'Transmit State'
             xl.Color = [0,0.3,0];
             xl.LabelVerticalAlignment = 'middle';
             xl.LabelHorizontalAlignment = 'right';
     elseif State_Name(k) == 'Initialisation State'
         xl.Color = 'c';
         xl.LabelVerticalAlignment = 'top';
         xl.LabelHorizontalAlignment = 'left';
     else
         xl.Color = 'k';
         xl.LabelVerticalAlignment = 'middle';
         xl.LabelHorizontalAlignment = 'left';
     end
end
title("Device Current Draw over a single cycle")
xlabel("Time (hh:mm:ss)");
ylabel("Current (mA)");
xlim([-minutes(15) hours(2)])
%calculate average current over a cycle
figure(2)
I_int = cumtrapz(seconds(T),CurrentmA);
I_avg = (max(I_int)-min(I_int))/(max(seconds(T))-min(seconds(T)));
T_cycle = max(seconds(T))-min(seconds(T));
I_avg_state = zeros(1,9);

I_state_duration = zeros(1,9);
for k = 2:10
index = find((T> T_State(k-1))&(T < T_State(k)));
I_state_duration(k-1) = (max(seconds(T(index)))-min(seconds(T(index))));
I_avg_state(k-1) = (max(I_int(index))-min(I_int(index)))/I_state_duration(k-1);
end

bar(I_avg_state);
set(gca, 'XTickLabel',State_Name)
set(gca, 'XTick',1:9)%unless you specify x-values in plot and bar
title("Graph showing the Average Current Consumption in each phase of the opperational Cycle");
xlabel("Buoy State")
ylabel("Active Current draw (mA)")
% Calaculate averages
