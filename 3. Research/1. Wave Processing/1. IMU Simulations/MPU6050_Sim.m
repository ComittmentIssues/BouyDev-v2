
%Simulate the output of an MPU6050 IMU. The device has a 3 axes
%Accelerometer and 3 axes Gyroscope. The device is a MEMs based IMU with an
%ADC connected to each channel. The adc outputs a 16 bit word representing
%a sample on a single axes

%Set up for the gyroscope is as follows:

gparams = gyroparams;
aparams = accelparams;

%Simulation Sampling Parameters:
Fs = 100;       % Sample Frequency (Hz)
Fw = 0.5;     % Wave Frequency (Hz)
Ts = 1/Fw;      % Sample Time (mins)
%N = Ts*60*Fs; % Number of Samples
N = Ts*Fs;
%Vectors
t = (0:1/Fs:(N-1)/Fs); %create linear time plot vector in multiples of 1/Fs
 Acc = zeros(N,3); % 3 - axis acceleration (m/s^2)
 angvel = zeros (N,3); % 3 - axis angular velocity (rad/s)
 
 Acc(:,2) =  sin(2*pi*Fw*t);
 angvel(:,1) = sin(2*pi*Fw*t);

 %Initialise IMU Object
 imu = imuSensor('accel-gyro', 'ReferenceFrame', 'ENU');
 imu.SampleRate =Fs;
 %get IMU Dataset
[accelData, gyroData] = imu(Acc, angvel);

figure(1)
plot(t, angvel(:,1), '--', t, gyroData(:,1))
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')
title('Ideal Gyroscope Data')
legend('x (ground truth)', 'x (gyroscope)')
figure(2)
plot(t, Acc(:,2), '--', t, -accelData(:,2))
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')
title('Ideal Accelerometer Data')
legend('x (ground truth)', 'x (accelerometer)')
