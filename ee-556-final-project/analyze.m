clc
clear
close all

%pkg load control

% load the data from a file
data = csvread("log.csv");

% separate out the columns into their own matrices
t = data(:, 1) / 1000;
vel = data(:, 2);
power = data(:, 3);

% plot normalized data
plot(t, vel ./ max(vel))
hold on
plot(t, power ./ max(power))
legend("Normalized Velocity", "Normalized Power", 'location', 'northwest')