%% Glavna skripta za pokretanje simulacije

clear; clc; close all;

% Dodaj staze do modela i kontrolera
addpath(genpath('src/model'));
addpath(genpath('src/controller'));

%% Postavke scenarija
fprintf('Odaberite scenarij:\n');
fprintf('1: Stabilizacija (start blizu vrha)\n');
fprintf('2: Swing-up (start s dna)\n');
scenario = input('Unesite broj scenarija: ');

%% Postavke simulacije
Ts = 0.05;      % Vrijeme koraka [s]

% Definicija ciljnog stanja (uspravna pozicija)
x_target = [0; 0; 0; 0]; 

if scenario == 1
    % SCENARIJ 1: Stabilizacija
    T_end = 10;     % Ukupno vrijeme simulacije [s]
    x0 = [0; 0.1; 0; 0]; % Početno stanje (mali pomak od vrha)
    Q = diag([10, 10, 1, 1]); % Težinska matrica za stanja
    R = 0.1; % Težinska matrica za ulaz
else
    % SCENARIJ 2: Swing-up
    T_end = 15;     % Ukupno vrijeme simulacije [s]
    x0 = [0; pi; 0; 0]; % Početno stanje (visak na dnu)
    Q = diag([1, 10, 0.1, 0.1]); % Težinska matrica (manji naglasak na poziciju)
    R = 0.01; % Težinska matrica za ulaz (dopuštamo veći ulaz)
end


%% Kreiraj model sustava
model = CartPendulumModel();

%% Postavke NMPC kontrolera
N = 30; % Horizont predikcije (malo duži za swing-up)
u_max = 20; % Ograničenje ulaza

% Kreiraj NMPC kontroler
controller = NMPCController(model, N, Q, R, u_max, Ts, x_target);

%% Kreiraj simulacijsko okruženje
sim = SimulationEnvironment(model, controller, Ts, T_end, x0);

%% Pokreni simulaciju
sim.run();

%% Prikaz rezultata
figure;
subplot(2,1,1);
plot(sim.results.T, sim.results.X);
hold on;
plot(sim.results.T, repmat(x_target', length(sim.results.T), 1), '--');
title('Stanja sustava');
legend('x', '\theta', 'v_x', '\omega', 'x_{ref}', '\theta_{ref}', 'v_{x,ref}', '\omega_{ref}');
grid on;

subplot(2,1,2);
plot(sim.results.T(1:end-1), sim.results.U);
title('Upravljački signal');
legend('u');
grid on;

%% Pokreni animaciju
sim.animate();
