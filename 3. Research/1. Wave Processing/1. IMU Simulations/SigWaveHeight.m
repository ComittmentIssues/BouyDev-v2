
% This is a function to get significant wave heights from a set of data
% sampled at a known frequency for a period of atleast 20 minutes.
% Assume the dominant wave frequency is 0.5Hz 
% Take the nyqist frequency to be 1Hz for ocean waves
% Parameters
T_sample = 20; %mins
f_sample = 5; %Hz
f_w = 0.2;
A_max = 3; %m/s-2
N = T_sample*60*f_sample; %Number of Samples
fn = f_sample/2; %Nyquist Frequency
[t,data] = generate_WaveData(T_sample,f_sample,f_w,A_max);
spec = fft(data)/N;

d_bins = reshape(data,20,300)';
t_bins = reshape(t,20,300)';
%% Double Integration For Displacement Data
y1 = cumtrapz(t_bins,d_bins);

y1 = detrend(y1,4);

%% Significant wave height calculation

PSD_EST = fft(y1)/N;
df = f_sample/N;
f = 0:df:f_sample-df;
R_w  = generate_Window(0.02,f_sample/2,f)';
PSD_FILT = R_w.*PSD_EST; 

Hs = mean(max(abs(PSD_FILT)))
%% Figures
figure(1);
subplot(2,1,1);
plot(t/60, data);
xlabel('Time (min)');
ylabel('Accelerometer Data (m/s^2)');
title('Wave Data');
subplot(2,1,2);
plot(t_bins/60,y1);
title('Filtered Time-series');
xlabel('t (min)');
ylabel('Displacement (m)');

figure(2);
subplot(3,1,1);
plot(f1,abs(spec));
subplot(3,1,2)
plot(f,abs(PSD_EST),f,abs(PSD_FILT));
subplot(3,1,3)
plot(f,R_w);

%% Functions
%generates a set of simulated wave data 
function [t,wd] = generate_WaveData(T_sample,f_sample,f_wave,A_wave)
N = T_sample*60*f_sample; %Number of Samples
t = [0:(1/f_sample):(N-1)/f_sample]; %time Vector
wd = A_wave*cos(2*pi*f_wave*t);%+rand(1, N);
end

%generate cosine taper function
function R = generate_Window(f1,f2,f)
R = zeros(1,length(f));
cond = ((f >= f1) & (f <=f2));
R(1,cond) = 4;
% R(1,cond)= (1/2)*(1 - cos(pi*(f(cond)-f1)/(f2-f1)))*(-1/(2*pi)^2).*(f(cond).^-2);
% cond = ((f >= f2) & (f <=fc));
% R(1,cond) = -(1/(2*pi)^2)*(f(cond).^-2);
end