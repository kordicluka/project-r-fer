%% Glavna skripta za pokretanje simulacije

clear; clc; close all;

% Dodaj staze do modela i kontrolera
addpath(genpath('src/model'));
addpath(genpath('src/controller'));

%% Postavke scenarija
fprintf('Odaberite scenarij:\n');
fprintf('1: Stabilizacija (start blizu vrha)\n');
fprintf('2: Swing-up (start s dna)\n');
fprintf('3: Odbacivanje poremećaja\n');
scenario = input('Unesite broj scenarija: ');

fprintf('\nOdaberite metodu integracije:\n');
fprintf('1: rk4 (default)\n');
fprintf('2: euler\n');
fprintf('3: ode45\n');
method_choice = input('Unesite broj metode: ');

if method_choice == 2
    integration_method = 'euler';
elif method_choice == 3
    integration_method = 'ode45';
else
    integration_method = 'rk4';
end

%% Postavke simulacije
Ts = 0.05;      % Vrijeme koraka [s]
disturbance = @(t) 0; % Default: bez poremećaja

% Definicija ciljnog stanja (uspravna pozicija)
x_target = [0; 0; 0; 0]; 

if scenario == 1
    % SCENARIJ 1: Stabilizacija
    T_end = 10;
    x0 = [0; 0.1; 0; 0];
    Q = diag([10, 10, 1, 1]);
    R = 0.1;
elif scenario == 2
    % SCENARIJ 2: Swing-up
    T_end = 15;
    x0 = [0; pi; 0; 0];
    Q = diag([1, 10, 0.1, 0.1]);
    R = 0.01;
else
    % SCENARIJ 3: Odbacivanje poremećaja
    T_end = 10;
    x0 = [0; 0; 0; 0]; % Start from stable position
    Q = diag([10, 10, 1, 1]);
    R = 0.1;
    % Define a disturbance pulse
    disturbance = @(t) 10 * (t > 5 && t < 5.2);
end


%% Kreiraj model sustava
model = CartPendulumModel();

%% Postavke NMPC kontrolera
N = 30; % Horizont predikcije
u_max = 20; % Ograničenje ulaza

% Kreiraj NMPC kontroler
controller = NMPCController(model, N, Q, R, u_max, Ts, x_target, integration_method);

%% Kreiraj simulacijsko okruženje
sim = SimulationEnvironment(model, controller, Ts, T_end, x0, disturbance, integration_method);

%% Pokreni simulaciju
sim.run();

%% Prikaz rezultata
figure('Name', 'Rezultati simulacije');

% Plot stanja
subplot(3,1,1);
plot(sim.results.X);
hold on;
plot(sim.results.X.Time, repmat(x_target', length(sim.results.X.Time), 1), '--');
title('Stanja sustava');
legend('x', '\theta', 'v_x', '\omega');
grid on;

% Plot ulaza
subplot(3,1,2);
plot(sim.results.U);
title('Upravljački signal');
legend('u');
grid on;

% Plot cijene
subplot(3,1,3);
plot(sim.results.Cost);
title('Cijena (Cost)');
legend('Cost');
grid on;


%% Pokreni animaciju
sim.animate();
