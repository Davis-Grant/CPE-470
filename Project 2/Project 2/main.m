%Grant Davis
%CPE470 Project 2: Potential Field Path Planning
%03/31/2022

%Initialize
clc,clear
close all

%Set parameters for simulation
n = 2; % Number of dimensions
delta_t = 0.05; % Set time step
t = 0:delta_t:10;% Set total simulation time
lambda = 8.5; % Set scaling factor of attractive potential field
vr_max = 50; % Set maximum of robot velocity
error = zeros (length(t) - 1, 1); % Set tracking error

%Set VIRTUAL TARGET
qv = zeros (length(t),n); %Initial positions of virtual target
pv = 1.2; %Set velocity of virtual target
theta_t = zeros (length(t),1); % Initial heading of the virtual target

%===========Set ROBOT =================
%Set initial state of robot (robot)
qr = zeros (length(t),n); %initial position of robot
v_rd =  zeros (length(t),1); %Initial velocity of robot
theta_r = zeros (length(t),1); % Initial heading of the robot

%Set relative states between robot and VIRTUAL TARGET
qrv = zeros (length(t),n); %Save relative positions between robot and virtual target
prv = zeros(length(t),n); %Save relative velocities between robot and virtual target

%Compute initial relative states between robot and virtual target
qrv(1,:) = qv(1,:) - qr(1,:);%Compute the initial relative position

%Compute the initial relative velocity
prv(1,:) = [pv*cos(theta_t(1))-v_rd(1)*cos(theta_r(1)),
pv*sin(theta_t(1))-v_rd(1)*sin(theta_r(1))];

qt_diff = zeros(length(t), n);
phi = zeros(length(t), 1);

%Set noise mean and standard deviation
noise_mean = 0.5;
noise_std = 0.5; %try 0.2/0.5

%=========MAIN PROGRAM==================
for i =2:length(t) 
%Trajectory
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Linear Trajectory without noise
qv_x = t(i);
qv_y = t(i);
qv(i,:) = [qv_x, qv_y]; %compute position of target

%Linear Trajectory with noise
% qv_x = t(i)+ noise_std * randn + noise_mean;
% qv_y = 4*t(i) + 10 + noise_std * randn + noise_mean;
% qv(i,:) = [qv_x, qv_y];  %compute position of target

%Sin Wave Trajectory without noise
% qv_x = t(i);
% qv_y = sin(t(i));
% qv(i,:) = [qv_x, qv_y]; %compute position of target

%Sin Wave Trajectory with noise
% qv_x = t(i) + noise_std * randn + noise_mean;
% qv_y = 4*sin(t(i) * 3) + 10 + noise_std * randn + noise_mean;
% qv(i,:) = [qv_x, qv_y];  %compute position of target
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Compute the target heading
qt_diff(i,:) =  qv(i,:)- qv(i-1,:);
theta_t(i) = atan2(qt_diff(i,2),qt_diff(i,1));
phi(i) = atan2(qrv(i - 1,2), qrv(i - 1,1));

%modeling robot velocity
v_rd(i) = sqrt((pv^2) + (2*lambda*norm(qrv(i - 1,:))*pv*abs(cos(theta_t(i)-phi(i)))) + ((norm(qrv(i - 1,:))*lambda)^2));
if v_rd(i) >= vr_max
   v_rd(i) = vr_max;
end

%modeling robot heading
theta_r(i) = phi(i) + asin((pv*sin(theta_t(i) - phi(i))/v_rd(i)));

%UPDATE position and velocity of robot
qr(i,:) = qr(i-1,:) + v_rd(i)*delta_t*[cos(theta_r(i-1)), sin(theta_r(i-1))]; 
qrv(i,:) = qv(i,:) - qr(i,:);
prv(i,:) = [pv*cos(theta_t(i))-v_rd(i)*cos(theta_r(i)), pv*sin(theta_t(i))-v_rd(i)*sin(theta_r(i))];
error(i) = norm(qv(i,:)-qr(i,:));
  
%plot postions qv of virtual target
plot(qv(:,1),qv(:,2),'r>')
hold on

%plot postions qv of robot
plot(qr(:,1),qr(:,2),'g>')
M = getframe(gca);
%mov = addframe(mov,M);  
end

%Plot results
figure(2), plot(error(2:length(t)), 'b.')
legend('Distance error between robot and virtual target')
figure(3), plot(v_rd, 'b')
legend('Robot velocity')
figure(4), plot(theta_r, '--b')
hold on
plot(theta_t, '-.r')
hold on
plot(phi, 'k')
legend('Robot orientation', 'Target orientation', 'Relative Orientation')
