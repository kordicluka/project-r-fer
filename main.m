function main(varargin)
%% MAIN - Glavna skripta/funkcija za pokretanje simulacije
%
% Može se koristiti na dva načina:
%
% 1. Interaktivni način (bez argumenata):
%    >> main
%    Pita korisnika za sve parametre
%
% 2. Argumenti (za automatizaciju):
%    >> main(scenario, method, 'x0', [0;0.1;0;0], 'disturbance', disturbance_fcn, ...)
%
% Parametri:
%   scenario      - Broj scenarija (1, 2, ili 3)
%   method        - Metoda integracije: 'rk4', 'euler', ili 'ode45'
%
% Opcionalni parametri (name-value pairs):
%   'x0'          - Početno stanje [px; theta; vx; omega] (4x1)
%   'T_end'       - Trajanje simulacije [s]
%   'Ts'          - Vremenski korak [s] (default: 0.05)
%   'N'           - Horizont predikcije (default: 30)
%   'u_max'       - Maksimalna sila [N] (default: 20)
%   'Q'           - Težinska matrica stanja (4x4)
%   'R'           - Težina upravljanja (scalar)
%   'disturbance' - Funkcija poremećaja @(t)
%   'x_target'    - Ciljna točka [px; theta; vx; omega] (default: [0;0;0;0])
%
% Primjeri:
%   main                                          % Interaktivni način
%   main(1, 'rk4')                                % Stabilizacija s rk4
%   main(2, 'euler', 'N', 20, 'Ts', 0.1)          % Swing-up s prilagođenim parametrima
%   main(1, 'rk4', 'x0', [0; 0.2; 0; 0])          % Stabilizacija s prilagođenim početkom
%   main(3, 'rk4', 'disturbance', @(t) 5*(t>3 & t<4))  % Prilagođeni poremećaj

clear; clc; close all;

% Dodaj staze do modela i kontrolera
addpath(genpath('src/model'));
addpath(genpath('src/controller'));
addpath(genpath('src/simulator'));

%% Parse input arguments
if nargin == 0
    % INTERAKTIVNI NAČIN - Pitaj korisnika za sve
    fprintf('=== NMPC Simulacija Kolica s Njihalom ===\n\n');

    % Scenarij
    fprintf('Odaberite scenarij:\n');
    fprintf('  1: Stabilizacija (start blizu vrha)\n');
    fprintf('  2: Swing-up (start s dna)\n');
    fprintf('  3: Odbacivanje poremećaja\n');
    scenario = input('Unesite broj scenarija (1-3): ');

    % Metoda integracije
    fprintf('\nOdaberite metodu integracije:\n');
    fprintf('  1: rk4 (default, brzo i precizno)\n');
    fprintf('  2: euler (sporo)\n');
    fprintf('  3: ode45 (adaptivno)\n');
    method_choice = input('Unesite broj metode (1-3): ');

    if method_choice == 2
        integration_method = 'euler';
    elseif method_choice == 3
        integration_method = 'ode45';
    else
        integration_method = 'rk4';
    end

    % Učitaj postavke za odabrani scenarij
    [T_end, x0, Q, R, disturbance] = getScenarioDefaults(scenario);

    % Postavi ostale parametre na defaultne vrijednosti
    N = 30;
    u_max = 20;
    Ts = 0.05;
    x_target = [0; 0; 0; 0];

    % Odredi ime scenarija
    scenario_names = {'Stabilizacija', 'Swing-up', 'Odbacivanje poremećaja'};
    scenario_name = scenario_names{scenario};

elseif nargin >= 2
    % ARGUMENTI NAČIN
    scenario = varargin{1};
    integration_method = varargin{2};

    % Parse optional name-value pairs
    p = inputParser;
    addParameter(p, 'x0', []);
    addParameter(p, 'T_end', []);
    addParameter(p, 'Ts', 0.05);
    addParameter(p, 'N', 30);
    addParameter(p, 'u_max', 20);
    addParameter(p, 'Q', []);
    addParameter(p, 'R', []);
    addParameter(p, 'disturbance', []);
    addParameter(p, 'x_target', [0; 0; 0; 0]);

    parse(p, varargin{3:end});
    custom_params = p.Results;

    % Load scenario defaults
    [T_end_default, x0_default, Q_default, R_default, disturbance_default] = getScenarioDefaults(scenario);

    % Apply custom parameters or defaults
    x0 = custom_params.x0;
    if isempty(x0), x0 = x0_default; end

    T_end = custom_params.T_end;
    if isempty(T_end), T_end = T_end_default; end

    Q = custom_params.Q;
    if isempty(Q), Q = Q_default; end

    R = custom_params.R;
    if isempty(R), R = R_default; end

    disturbance = custom_params.disturbance;
    if isempty(disturbance), disturbance = disturbance_default; end

    Ts = custom_params.Ts;
    N = custom_params.N;
    u_max = custom_params.u_max;
    x_target = custom_params.x_target;

    % Determine scenario name
    scenario_names = {'Stabilizacija', 'Swing-up', 'Odbacivanje poremećaja'};
    scenario_name = scenario_names{scenario};
else
    error('Morate navesti barem scenarij i metodu: main(scenario, method)');
end

%% Validate inputs
if ~ismember(scenario, [1, 2, 3])
    error('Scenarij mora biti 1, 2, ili 3');
end
if ~ismember(integration_method, {'rk4', 'euler', 'ode45'})
    error('Metoda mora biti ''rk4'', ''euler'', ili ''ode45''');
end

%% Display configuration
fprintf('\n=== Konfiguracija Simulacije ===\n');
fprintf('Scenarij: %s\n', scenario_name);
fprintf('Metoda integracije: %s\n', integration_method);
fprintf('Početno stanje x0: [%.3f, %.4f, %.3f, %.3f]\n', x0);
fprintf('  (pozicija=%.3f m, kut=%.1f°, brzina=%.3f m/s, kutna_brzina=%.3f rad/s)\n', ...
        x0(1), rad2deg(x0(2)), x0(3), x0(4));
fprintf('Trajanje: %.1f s, Ts = %.3f s\n', T_end, Ts);
fprintf('Horizont: N = %d\n', N);
fprintf('Ograničenje ulaza: u_max = %.1f N\n', u_max);
fprintf('=====================================\n\n');

%% Kreiraj model sustava
model = CartPendulumModel();

%% Kreiraj NMPC kontroler
controller = NMPCController(model, N, Q, R, u_max, Ts, x_target, integration_method);

%% Kreiraj simulacijsko okruženje
sim = SimulationEnvironment(model, controller, Ts, T_end, x0, disturbance, integration_method);

%% Pokreni simulaciju
fprintf('Pokretanje simulacije...\n');
sim.run();
fprintf('Simulacija završena!\n\n');

%% Prikaz rezultata
figure('Name', sprintf('Rezultati - %s (%s)', scenario_name, integration_method));

% Plot stanja
subplot(3,1,1);
plot(sim.results.X);
hold on;
plot(sim.results.X.Time, repmat(x_target', length(sim.results.X.Time), 1), '--', 'LineWidth', 1.5);
title('Stanja sustava');
legend('x [m]', '\theta [rad]', 'v_x [m/s]', '\omega [rad/s]', 'Location', 'best');
xlabel('Vrijeme [s]');
ylabel('Vrijednost');
grid on;

% Plot ulaza
subplot(3,1,2);
plot(sim.results.U, 'LineWidth', 1.5);
hold on;
yline(u_max, 'r--', 'LineWidth', 1);
yline(-u_max, 'r--', 'LineWidth', 1);
title('Upravljački signal');
legend('u [N]', sprintf('Ograničenje (±%.0f N)', u_max), 'Location', 'best');
xlabel('Vrijeme [s]');
ylabel('Sila [N]');
grid on;

% Plot cijene
subplot(3,1,3);
plot(sim.results.Cost, 'LineWidth', 1.5);
title('Cijena (Cost Function)');
xlabel('Vrijeme [s]');
ylabel('Vrijednost');
grid on;

%% Pokreni animaciju
fprintf('Pokretanje animacije...\n');
sim.animate();

end

%% Helper function to get scenario defaults
function [T_end, x0, Q, R, disturbance] = getScenarioDefaults(scenario)
    if scenario == 1
        % SCENARIJ 1: Stabilizacija
        T_end = 10;
        x0 = [0; 0.1; 0; 0];
        Q = diag([10, 10, 1, 1]);
        R = 0.1;
        disturbance = @(t) 0;
    elseif scenario == 2
        % SCENARIJ 2: Swing-up
        T_end = 15;
        x0 = [0; pi; 0; 0];
        Q = diag([1, 10, 0.1, 0.1]);
        R = 0.01;
        disturbance = @(t) 0;
    else
        % SCENARIJ 3: Odbacivanje poremećaja
        T_end = 10;
        x0 = [0; 0; 0; 0];
        Q = diag([10, 10, 1, 1]);
        R = 0.1;
        disturbance = @(t) 10 * (t > 5 & t < 5.2);
    end
end
