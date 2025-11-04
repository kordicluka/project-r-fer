%% Glavna skripta za pokretanje simulacije

clear; clc; close all;

% Dodaj staze do modela i kontrolera
addpath(genpath('src/model'));
addpath(genpath('src/controller'));

%% Postavke simulacije
Ts = 0.05;      % Vrijeme koraka [s]
T_end = 10;     % Ukupno vrijeme simulacije [s]
x0 = [0; 0.1; 0; 0]; % Početno stanje [x; theta; vx; omega]

%% Kreiraj model sustava
model = CartPendulumModel();

%% Postavke NMPC kontrolera
N = 20; % Horizont predikcije
Q = diag([10, 10, 1, 1]); % Težinska matrica za stanja
R = 0.1; % Težinska matrica za ulaz
u_max = 20; % Ograničenje ulaza

% Kreiraj NMPC kontroler
controller = NMPCController(model, N, Q, R, u_max, Ts);

%% Kreiraj simulacijsko okruženje
sim = SimulationEnvironment(model, controller, Ts, T_end, x0);

%% Pokreni simulaciju
sim.run();

%% Prikaz rezultata
figure;
subplot(2,1,1);
plot(sim.results.T, sim.results.X);
title('Stanja sustava');
legend('x', '\theta', 'v_x', '\omega');
grid on;

subplot(2,1,2);
plot(sim.results.T(1:end-1), sim.results.U);
title('Upravljački signal');
legend('u');
grid on;

%% Pokreni animaciju
sim.animate();
