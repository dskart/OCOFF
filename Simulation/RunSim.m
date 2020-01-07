function [] = RunSim(lambda_in)
close all
%% VARIBALES

delta_t = 0.01;
total_time = 10;

m = 0.001;

desired_position_period = 3/total_time*pi;
desired_position_offset = 0;
desired_position_amplitude = 1;
simulation_position_noise_var = 0.5 * desired_position_amplitude;

oscillating_Kp_period = 1/(total_time)*pi;
oscillating_Kp_amplitude = 10;
oscillating_Kp_offset = oscillating_Kp_amplitude + 0.5;

%% INIT OSCILLATION VALUES
oscillating_postion = Oscillate(desired_position_amplitude,desired_position_period, desired_position_offset, total_time,delta_t);
oscillating_Kp = Oscillate(oscillating_Kp_amplitude, oscillating_Kp_period, oscillating_Kp_offset , total_time, delta_t);

%Optimal Lambda Constants
L = oscillating_Kp_amplitude * oscillating_Kp_period;
p0 = 10; 
delta0 = 0;
info_state_size = 1;
N = 5;
persistent_excited_avg = mean((10*randn())^2);
alpha = persistent_excited_avg;
beta = persistent_excited_avg;
    

c = oscillating_Kp;
num_samples = length(oscillating_postion)-1;


%% LOOP

true_system_torque_history = zeros(1,num_samples);
required_system_torque_history = zeros(1, num_samples);

true_input_torque_history = zeros(1,num_samples);
required_input_torque_history = zeros(1,num_samples);
spring_torques_history = zeros(1,num_samples);

velocities_history = zeros(1,num_samples);
position_history = zeros(1,num_samples);
samples_delta_t = 0:delta_t:total_time;

scaled_position_difference_history = zeros(1,num_samples);
Kp_history = zeros(1,num_samples);

lambda = zeros(1,num_samples);

new_position = oscillating_postion(1) + randn()*delta0;
estimate_Kp = oscillating_Kp(1);
P = p0;

for i = 1:num_samples
    Kp_history(i) = estimate_Kp;
    current_Kp = oscillating_Kp(i);

    noise = simulation_position_noise_var*randn();
    current_pos = new_position;
    position_history(i) = current_pos; %+ noise;
    measured_pos = current_pos + noise;
    
    % velocity estimate from position data
    if i == 1
        current_velocity = 0;
    else
        current_velocity = (current_pos - position_history(i-1))/ delta_t;
    end
    
    velocities_history(i) = current_velocity;

    wanted_position = oscillating_postion(i+1); %get the wanted position
    
    %calculate the new torque
    input_torque = (CalculateTorque(estimate_Kp, wanted_position, measured_pos, current_velocity, delta_t, m));
    needed_torque = (CalculateTorque(current_Kp, wanted_position, current_pos, current_velocity, delta_t, m));
    
    actual_torque = input_torque - (current_Kp*current_pos);%spring force
    
    spring_torques_history(i) = current_Kp*current_pos;
    true_system_torque_history(i) = actual_torque;
    required_system_torque_history(i) = needed_torque - (current_Kp*current_pos);
 
    true_input_torque_history(i) = input_torque;
    required_input_torque_history(i) = needed_torque;
    
    new_position = NewPositionFromTorque(actual_torque, current_pos, current_velocity, delta_t, m);
    scaled_position_difference_history(i) = -(m/delta_t^2 * (new_position - current_pos - current_velocity * delta_t - delta_t^2/m * input_torque));
    

    if isstring(lambda_in)
        lambda(i) = ComputeOptimalLambda(alpha, beta, p0, delta0, info_state_size, N, i+N, simulation_position_noise_var, L);
    else
        lambda(i) = lambda_in;
    end
    [estimate_Kp, P] = RLS_With_Forgetting(estimate_Kp, P, current_pos, scaled_position_difference_history(i), lambda(i)); % calculate new c
    
    c(i) = estimate_Kp;
    
end

%% PLOTTING DATA
sz = [800 1800]; % figure size
screensize = get(groot,'ScreenSize');
xpos = ceil((screensize(3)-sz(2))/2); % center the figure on the
ypos = ceil((screensize(4)-sz(1))/2); % center the figure on the
figHandle=figure('position',[xpos, ypos, sz(2), sz(1)]);

subplot(5,1,1)
plot(samples_delta_t(1,1:end-1), oscillating_postion(1,1:end-1), "LineWidth", 3)
title("desired position");
hold on
plot(samples_delta_t(1,1:end-1), position_history, 'o');
legend("Desired Position", "True Position");

subplot(5,1,2)
plot(samples_delta_t(1,1:end-1), oscillating_Kp(1,1:end-1), "LineWidth", 3)
hold on
plot(samples_delta_t(1,1:end-1), c(1,1:end-1), 'o')
title("Kp estimate");
legend("Desired Kp", "Estimated Kp");

subplot(5,1,3)
plot(samples_delta_t(1,1:end-1), required_system_torque_history)
title("System Torques");
hold on
plot(samples_delta_t(1,1:end-1), true_system_torque_history, 'o')
legend("Desired Total Torque", "True Total Torque");

subplot(5,1,4)
plot(samples_delta_t(1,1:end-1), true_input_torque_history)
title("Input Torques");
hold on
plot(samples_delta_t(1,1:end-1), required_input_torque_history, 'o')
legend("Input Torque Desired", "True Torque Inputed");

subplot(5,1,5)
plot(samples_delta_t(1,1:end-1), lambda);
title("Forgetting Factor")


%get the directory of your input files:
pathname = strcat(pwd, strcat( '/Sim_Results/Lambda',num2str(lambda_in)));
mkdir(pathname)

%use that when you save
save(fullfile(pathname, 'lambda.mat'), 'lambda');
save(fullfile(pathname, 'oscillating_position.mat'), 'oscillating_postion')
save(fullfile(pathname, 'oscillating_Kp.mat'), 'oscillating_Kp')
save(fullfile(pathname, 'position_history.mat'), 'position_history')
save(fullfile(pathname, 'required_system_torque_history.mat'), 'required_system_torque_history')
save(fullfile(pathname, 'true_system_torque_history.mat'), 'true_system_torque_history')
save(fullfile(pathname, 'true_input_torque_history.mat'), 'true_input_torque_history')
save(fullfile(pathname, 'required_input_torque_history.mat'), 'required_input_torque_history')
saveas(figHandle, fullfile(pathname, 'figure.jpg'))

end
