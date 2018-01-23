clear
close all
clc

addpath('tools')

landmarks = read_world('./data/world.dat');
sensor_data = read_data('./data/sensor_data.dat');

INF = 1000;
N = size(landmarks, 2);

observed_landmarks = false(1, N);

mu = zeros(2*N + 3, 1);
robot_sigma = zeros(3);
robot_landmarks_sigma = zeros(3, 2*N);
landmarks_sigma = INF*eye(2*N);
sigma = [[robot_sigma, robot_landmarks_sigma];...
    [robot_landmarks_sigma',landmarks_sigma]];

showGui = true;

for t = 1:size(sensor_data.timestep, 2)
    [mu, sigma] = prediction(mu, sigma, sensor_data.timestep(t).odometry);
    
    [mu, sigma, observed_landmarks] = correction(mu, sigma, ...
        sensor_data.timestep(t).sensor, observed_landmarks);
    
    plot_state(mu, sigma, landmarks, t, observed_landmarks, ...
        sensor_data.timestep(t).sensor, showGui);
    disp('Current state:');
    disp('mu = '), disp(mu)
end

disp('Final system covariance matrix: '), disp(sigma);

disp('Final robot pose: ');
disp('mu_robot = '), disp(mu(1:3)), disp('sigma_robot = '), disp(sigma(1:3, 1:3));
rmpath('tools');