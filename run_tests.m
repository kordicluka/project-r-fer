%% RUN_TESTS - Automatska skripta za testiranje NMPC kontrolera
%
% Ova skripta izvršava seriju testova s različitim parametrima i generira
% rezultate za izvještaj (grafovi, tablice).
%
% Testovi:
%   1. Utjecaj horizonta predikcije (N)
%   2. Utjecaj vremena diskretizacije (Ts)
%   3. Utjecaj ograničenja ulaza (u_max)
%   4. Utjecaj težinskih matrica (Q, R)
%   5. Usporedba metoda integracije
%   6. Usporedba diskretnog i kontinuiranog modela
%
% Rezultati se spremaju u strukturu 'test_results' i folder 'results/'
%
% Pokretanje:
%   >> run_tests          % Pokreni sve testove
%   >> run_tests(1)       % Pokreni samo test 1
%   >> run_tests([1,3,5]) % Pokreni testove 1, 3 i 5

function run_tests(test_ids)

    % Ako nisu specificirani testovi, pokreni sve
    if nargin == 0
        test_ids = 1:6;
    end

    % Inicijalizacija (bez clear da ne obriše test_ids)
    clc; close all;
    addpath(genpath('src/model'));
    addpath(genpath('src/controller'));
    addpath(genpath('src/simulator'));

    % Kreiraj folder za rezultate
    if ~exist('results', 'dir')
        mkdir('results');
    end

    % Definiraj nazive foldera za svaki test
    test_folders = {
        'results/test1_horizon_N', ...
        'results/test2_sample_time_Ts', ...
        'results/test3_input_constraint_umax', ...
        'results/test4_weight_matrices_QR', ...
        'results/test5_integration_methods', ...
        'results/test6_discrete_vs_continuous'
    };

    % Kreiraj foldere za sve testove koji će se pokrenuti
    for i = test_ids
        if ~exist(test_folders{i}, 'dir')
            mkdir(test_folders{i});
        end
    end

    % Struktura za spremanje svih rezultata
    all_results = struct();

    fprintf('╔════════════════════════════════════════════════════════════╗\n');
    fprintf('║          NMPC TESTIRANJE - CART-PENDULUM SUSTAV            ║\n');
    fprintf('╚════════════════════════════════════════════════════════════╝\n\n');

    %% TEST 1: Utjecaj horizonta predikcije (N)
    if ismember(1, test_ids)
        fprintf('\n▶ TEST 1: Utjecaj horizonta predikcije (N)\n');
        fprintf('─────────────────────────────────────────────\n');

        N_values = [10, 20, 30, 50];
        results_N = cell(length(N_values), 1);
        metrics_N = cell(length(N_values), 1);

        for i = 1:length(N_values)
            N = N_values(i);
            fprintf('  Testiram N = %d... ', N);

            [results_N{i}, metrics_N{i}] = runSingleTest(...
                'N', N, 'Ts', 0.05, 'u_max', 20, ...
                'Q', diag([10,10,1,1]), 'R', 0.1, ...
                'x0', [0; 0.1; 0; 0], 'T_end', 10, ...
                'integration_method', 'rk4', 'use_continuous_model', false);

            fprintf('ISE=%.4f, Ts_solve=%.2fms\n', ...
                metrics_N{i}.ISE_total, metrics_N{i}.MeanSolveTime*1000);
        end

        % Spremi rezultate
        all_results.test1.N_values = N_values;
        all_results.test1.results = results_N;
        all_results.test1.metrics = metrics_N;

        % Generiraj graf
        plotTest1(N_values, metrics_N);
        saveas(gcf, fullfile(test_folders{1}, 'graf_horizont_N.png'));
        fprintf('  Graf spremljen: %s/graf_horizont_N.png\n', test_folders{1});

        % Spremi podatke testa u zaseban folder
        test1_data = struct('N_values', N_values, 'metrics', {metrics_N}, 'results', {results_N});
        save(fullfile(test_folders{1}, 'test1_data.mat'), 'test1_data');
        writeTestTable(test_folders{1}, 'test1_results.txt', ...
            {'N', 'ISE', 'IAE', 'SettlingTime_s', 'SolveTime_ms', 'Iterations'}, ...
            N_values, metrics_N, {'ISE_total', 'IAE_total', 'SettlingTime_theta', 'MeanSolveTime', 'MeanIterations'}, ...
            [1, 1, 1, 1000, 1]);  % Multipliers (SolveTime needs *1000 for ms)
    end

    %% TEST 2: Utjecaj vremena diskretizacije (Ts)
    if ismember(2, test_ids)
        fprintf('\n▶ TEST 2: Utjecaj vremena diskretizacije (Ts)\n');
        fprintf('──────────────────────────────────────────────\n');

        Ts_values = [0.02, 0.05, 0.1, 0.15];
        results_Ts = cell(length(Ts_values), 1);
        metrics_Ts = cell(length(Ts_values), 1);

        for i = 1:length(Ts_values)
            Ts = Ts_values(i);
            % Održi isti vremenski horizont (1.5s)
            N = round(1.5 / Ts);
            fprintf('  Testiram Ts = %.2fs (N=%d)... ', Ts, N);

            [results_Ts{i}, metrics_Ts{i}] = runSingleTest(...
                'N', N, 'Ts', Ts, 'u_max', 20, ...
                'Q', diag([10,10,1,1]), 'R', 0.1, ...
                'x0', [0; 0.1; 0; 0], 'T_end', 10, ...
                'integration_method', 'rk4', 'use_continuous_model', false);

            fprintf('ISE=%.4f, Settling=%.2fs\n', ...
                metrics_Ts{i}.ISE_total, metrics_Ts{i}.SettlingTime_theta);
        end

        all_results.test2.Ts_values = Ts_values;
        all_results.test2.results = results_Ts;
        all_results.test2.metrics = metrics_Ts;

        plotTest2(Ts_values, metrics_Ts);
        saveas(gcf, fullfile(test_folders{2}, 'graf_sample_time_Ts.png'));
        fprintf('  Graf spremljen: %s/graf_sample_time_Ts.png\n', test_folders{2});

        % Spremi podatke testa
        test2_data = struct('Ts_values', Ts_values, 'metrics', {metrics_Ts}, 'results', {results_Ts});
        save(fullfile(test_folders{2}, 'test2_data.mat'), 'test2_data');
        writeTestTable2(test_folders{2}, 'test2_results.txt', Ts_values, metrics_Ts);
    end

    %% TEST 3: Utjecaj ograničenja ulaza (u_max)
    if ismember(3, test_ids)
        fprintf('\n▶ TEST 3: Utjecaj ograničenja ulaza (u_max)\n');
        fprintf('────────────────────────────────────────────\n');

        u_max_values = [5, 10, 20, 50];
        results_umax = cell(length(u_max_values), 1);
        metrics_umax = cell(length(u_max_values), 1);

        for i = 1:length(u_max_values)
            u_max = u_max_values(i);
            fprintf('  Testiram u_max = %d N... ', u_max);

            [results_umax{i}, metrics_umax{i}] = runSingleTest(...
                'N', 30, 'Ts', 0.05, 'u_max', u_max, ...
                'Q', diag([10,10,1,1]), 'R', 0.1, ...
                'x0', [0; 0.3; 0; 0], 'T_end', 10, ...  % Veći početni kut
                'integration_method', 'rk4', 'use_continuous_model', false);

            fprintf('ISE=%.4f, MaxU=%.2fN\n', ...
                metrics_umax{i}.ISE_total, metrics_umax{i}.MaxControlInput);
        end

        all_results.test3.u_max_values = u_max_values;
        all_results.test3.results = results_umax;
        all_results.test3.metrics = metrics_umax;

        plotTest3(u_max_values, metrics_umax, results_umax);
        saveas(gcf, fullfile(test_folders{3}, 'graf_input_constraint.png'));
        fprintf('  Graf spremljen: %s/graf_input_constraint.png\n', test_folders{3});

        % Spremi podatke testa
        test3_data = struct('u_max_values', u_max_values, 'metrics', {metrics_umax}, 'results', {results_umax});
        save(fullfile(test_folders{3}, 'test3_data.mat'), 'test3_data');
        writeTestTable3(test_folders{3}, 'test3_results.txt', u_max_values, metrics_umax);
    end

    %% TEST 4: Utjecaj težinskih matrica (Q, R)
    if ismember(4, test_ids)
        fprintf('\n▶ TEST 4: Utjecaj težinskih matrica (Q, R)\n');
        fprintf('───────────────────────────────────────────\n');

        % Različiti omjeri Q/R
        QR_configs = {
            struct('Q', diag([10,10,1,1]), 'R', 0.1, 'name', 'Standardno'),
            struct('Q', diag([10,10,1,1]), 'R', 1.0, 'name', 'Visoki R'),
            struct('Q', diag([50,50,5,5]), 'R', 0.01, 'name', 'Agresivno'),
            struct('Q', diag([1,50,0.1,0.1]), 'R', 0.1, 'name', 'Prioritet kut')
        };

        results_QR = cell(length(QR_configs), 1);
        metrics_QR = cell(length(QR_configs), 1);

        for i = 1:length(QR_configs)
            cfg = QR_configs{i};
            fprintf('  Testiram %s... ', cfg.name);

            [results_QR{i}, metrics_QR{i}] = runSingleTest(...
                'N', 30, 'Ts', 0.05, 'u_max', 20, ...
                'Q', cfg.Q, 'R', cfg.R, ...
                'x0', [0; 0.2; 0; 0], 'T_end', 10, ...
                'integration_method', 'rk4', 'use_continuous_model', false);

            fprintf('ISE=%.4f, Effort=%.2f\n', ...
                metrics_QR{i}.ISE_total, metrics_QR{i}.TotalControlEffort);
        end

        all_results.test4.QR_configs = QR_configs;
        all_results.test4.results = results_QR;
        all_results.test4.metrics = metrics_QR;

        plotTest4(QR_configs, metrics_QR, results_QR);
        saveas(gcf, fullfile(test_folders{4}, 'graf_weight_matrices.png'));
        fprintf('  Graf spremljen: %s/graf_weight_matrices.png\n', test_folders{4});

        % Spremi podatke testa
        test4_data = struct('QR_configs', {QR_configs}, 'metrics', {metrics_QR}, 'results', {results_QR});
        save(fullfile(test_folders{4}, 'test4_data.mat'), 'test4_data');
        writeTestTable4(test_folders{4}, 'test4_results.txt', QR_configs, metrics_QR);
    end

    %% TEST 5: Usporedba metoda integracije
    if ismember(5, test_ids)
        fprintf('\n▶ TEST 5: Usporedba metoda integracije\n');
        fprintf('───────────────────────────────────────\n');

        methods = {'euler', 'rk4', 'ode45'};
        results_method = cell(length(methods), 1);
        metrics_method = cell(length(methods), 1);

        for i = 1:length(methods)
            method = methods{i};
            fprintf('  Testiram %s... ', method);

            [results_method{i}, metrics_method{i}] = runSingleTest(...
                'N', 30, 'Ts', 0.05, 'u_max', 20, ...
                'Q', diag([10,10,1,1]), 'R', 0.1, ...
                'x0', [0; 0.1; 0; 0], 'T_end', 10, ...
                'integration_method', method, 'use_continuous_model', false);

            fprintf('ISE=%.4f, Ts_solve=%.2fms\n', ...
                metrics_method{i}.ISE_total, metrics_method{i}.MeanSolveTime*1000);
        end

        all_results.test5.methods = methods;
        all_results.test5.results = results_method;
        all_results.test5.metrics = metrics_method;

        plotTest5(methods, metrics_method, results_method);
        saveas(gcf, fullfile(test_folders{5}, 'graf_integration_methods.png'));
        fprintf('  Graf spremljen: %s/graf_integration_methods.png\n', test_folders{5});

        % Spremi podatke testa
        test5_data = struct('methods', {methods}, 'metrics', {metrics_method}, 'results', {results_method});
        save(fullfile(test_folders{5}, 'test5_data.mat'), 'test5_data');
        writeTestTable5(test_folders{5}, 'test5_results.txt', methods, metrics_method);
    end

    %% TEST 6: Usporedba diskretnog i kontinuiranog modela
    if ismember(6, test_ids)
        fprintf('\n▶ TEST 6: Usporedba diskretnog i kontinuiranog modela\n');
        fprintf('─────────────────────────────────────────────────────\n');

        model_types = {
            struct('continuous', false, 'name', 'Diskretni (RK4)'),
            struct('continuous', true, 'name', 'Kontinuirani (nlmpc)')
        };

        results_model = cell(length(model_types), 1);
        metrics_model = cell(length(model_types), 1);

        for i = 1:length(model_types)
            cfg = model_types{i};
            fprintf('  Testiram %s... ', cfg.name);

            [results_model{i}, metrics_model{i}] = runSingleTest(...
                'N', 30, 'Ts', 0.05, 'u_max', 20, ...
                'Q', diag([10,10,1,1]), 'R', 0.1, ...
                'x0', [0; 0.1; 0; 0], 'T_end', 10, ...
                'integration_method', 'rk4', 'use_continuous_model', cfg.continuous);

            fprintf('ISE=%.4f, Ts_solve=%.2fms\n', ...
                metrics_model{i}.ISE_total, metrics_model{i}.MeanSolveTime*1000);
        end

        all_results.test6.model_types = model_types;
        all_results.test6.results = results_model;
        all_results.test6.metrics = metrics_model;

        plotTest6(model_types, metrics_model, results_model);
        saveas(gcf, fullfile(test_folders{6}, 'graf_discrete_vs_continuous.png'));
        fprintf('  Graf spremljen: %s/graf_discrete_vs_continuous.png\n', test_folders{6});

        % Spremi podatke testa
        test6_data = struct('model_types', {model_types}, 'metrics', {metrics_model}, 'results', {results_model});
        save(fullfile(test_folders{6}, 'test6_data.mat'), 'test6_data');
        writeTestTable6(test_folders{6}, 'test6_results.txt', model_types, metrics_model);
    end

    %% Generiraj tablicu sažetka
    fprintf('\n▶ Generiranje tablice sažetka...\n');
    generateSummaryTable(all_results);

    %% Spremi sve rezultate
    save('results/all_test_results.mat', 'all_results');
    fprintf('\n✓ Svi rezultati spremljeni u results/all_test_results.mat\n');

    fprintf('\n╔════════════════════════════════════════════════════════════╗\n');
    fprintf('║                    TESTIRANJE ZAVRŠENO                     ║\n');
    fprintf('╚════════════════════════════════════════════════════════════╝\n');
end

%% ═══════════════════════════════════════════════════════════════════════
%  POMOĆNE FUNKCIJE
%% ═══════════════════════════════════════════════════════════════════════

function [results, metrics] = runSingleTest(varargin)
    %% Pokreni jedan test s danim parametrima

    % Parse parametara
    p = inputParser;
    addParameter(p, 'N', 30);
    addParameter(p, 'Ts', 0.05);
    addParameter(p, 'u_max', 20);
    addParameter(p, 'Q', diag([10,10,1,1]));
    addParameter(p, 'R', 0.1);
    addParameter(p, 'x0', [0;0.1;0;0]);
    addParameter(p, 'x_target', [0;0;0;0]);
    addParameter(p, 'T_end', 10);
    addParameter(p, 'integration_method', 'rk4');
    addParameter(p, 'use_continuous_model', false);
    parse(p, varargin{:});
    cfg = p.Results;

    % Kreiraj komponente
    model = CartPendulumModel();
    controller = NMPCController(model, cfg.N, cfg.Q, cfg.R, cfg.u_max, ...
        cfg.Ts, cfg.integration_method, cfg.use_continuous_model);
    sim = SimulationEnvironment(model, controller, cfg.T_end, cfg.x0, cfg.x_target);

    % Pokreni simulaciju (bez waitbar-a)
    runSimulationQuiet(sim);

    % Vrati rezultate
    results = sim.results;
    metrics = results.Metrics;
end

function runSimulationQuiet(sim)
    %% Pokreni simulaciju bez waitbar-a (za batch testiranje)

    Ts = sim.controller.Ts;
    integration_method = sim.controller.integration_method;
    x = sim.x0;

    time_vec = (0:Ts:sim.T_end)';
    nSteps = length(time_vec);
    X_vec = zeros(nSteps, length(x));
    U_vec = zeros(nSteps-1, 1);
    Cost_vec = zeros(nSteps-1, 1);
    SolveTime_vec = zeros(nSteps-1, 1);
    Iterations_vec = zeros(nSteps-1, 1);

    X_vec(1, :) = x';
    yref = sim.x_target';

    for i = 1:nSteps-1
        t = time_vec(i);
        sim.param_change(t, sim.model);

        [u, cost, opt_info] = sim.controller.computeControlAction(x, yref);

        d_force = sim.disturbance_force(t);
        u_total = u + d_force;
        d_torque = sim.disturbance_torque(t);

        x_next = sim.simulateStepWithTorque(x, u_total, Ts, integration_method, d_torque);
        x = x_next;

        X_vec(i+1, :) = x';
        U_vec(i, :) = u_total';
        Cost_vec(i) = cost;
        SolveTime_vec(i) = opt_info.SolveTime;
        Iterations_vec(i) = opt_info.Iterations;
    end

    sim.results.X = timeseries(X_vec, time_vec, 'Name', 'Stanja');
    sim.results.U = timeseries(U_vec, time_vec(1:end-1), 'Name', 'Ulaz');
    sim.results.Cost = timeseries(Cost_vec, time_vec(1:end-1), 'Name', 'Cijena');
    sim.results.SolveTime = timeseries(SolveTime_vec, time_vec(1:end-1), 'Name', 'VrijemeRjesavanja');
    sim.results.Iterations = timeseries(Iterations_vec, time_vec(1:end-1), 'Name', 'BrojIteracija');
    sim.results.Metrics = sim.calculateMetrics();
end

%% ═══════════════════════════════════════════════════════════════════════
%  FUNKCIJE ZA GRAFOVE
%% ═══════════════════════════════════════════════════════════════════════

function plotTest1(N_values, metrics)
    figure('Name', 'Test 1: Horizont predikcije N', 'Position', [100 100 1200 400]);

    % ISE vs N
    subplot(1,3,1);
    ISE_vals = cellfun(@(m) m.ISE_total, metrics);
    bar(categorical(string(N_values)), ISE_vals);
    xlabel('Horizont N'); ylabel('ISE'); title('ISE vs Horizont');
    grid on;

    % Vrijeme rješavanja vs N
    subplot(1,3,2);
    time_vals = cellfun(@(m) m.MeanSolveTime*1000, metrics);
    bar(categorical(string(N_values)), time_vals);
    xlabel('Horizont N'); ylabel('Vrijeme [ms]'); title('Vrijeme rješavanja vs Horizont');
    grid on;

    % Settling time vs N
    subplot(1,3,3);
    settling_vals = cellfun(@(m) m.SettlingTime_theta, metrics);
    bar(categorical(string(N_values)), settling_vals);
    xlabel('Horizont N'); ylabel('Vrijeme [s]'); title('Settling time vs Horizont');
    grid on;

    sgtitle('Test 1: Utjecaj horizonta predikcije (N)');
end

function plotTest2(Ts_values, metrics)
    figure('Name', 'Test 2: Vrijeme diskretizacije Ts', 'Position', [100 100 1200 400]);

    labels = arrayfun(@(x) sprintf('%.2fs', x), Ts_values, 'UniformOutput', false);

    subplot(1,3,1);
    ISE_vals = cellfun(@(m) m.ISE_total, metrics);
    bar(categorical(labels), ISE_vals);
    xlabel('Ts'); ylabel('ISE'); title('ISE vs Ts');
    grid on;

    subplot(1,3,2);
    settling_vals = cellfun(@(m) m.SettlingTime_theta, metrics);
    bar(categorical(labels), settling_vals);
    xlabel('Ts'); ylabel('Vrijeme [s]'); title('Settling time vs Ts');
    grid on;

    subplot(1,3,3);
    effort_vals = cellfun(@(m) m.TotalControlEffort, metrics);
    bar(categorical(labels), effort_vals);
    xlabel('Ts'); ylabel('Napor'); title('Upravljački napor vs Ts');
    grid on;

    sgtitle('Test 2: Utjecaj vremena diskretizacije (Ts)');
end

function plotTest3(u_max_values, metrics, results)
    figure('Name', 'Test 3: Ograničenje ulaza u_max', 'Position', [100 100 1200 800]);

    labels = arrayfun(@(x) sprintf('%dN', x), u_max_values, 'UniformOutput', false);

    % Gornji red: metrike
    subplot(2,3,1);
    ISE_vals = cellfun(@(m) m.ISE_total, metrics);
    bar(categorical(labels), ISE_vals);
    xlabel('u_{max}'); ylabel('ISE'); title('ISE vs u_{max}');
    grid on;

    subplot(2,3,2);
    settling_vals = cellfun(@(m) m.SettlingTime_theta, metrics);
    bar(categorical(labels), settling_vals);
    xlabel('u_{max}'); ylabel('Vrijeme [s]'); title('Settling time vs u_{max}');
    grid on;

    subplot(2,3,3);
    max_u_vals = cellfun(@(m) m.MaxControlInput, metrics);
    bar(categorical(labels), max_u_vals);
    xlabel('u_{max}'); ylabel('Max |u| [N]'); title('Maksimalni ulaz vs u_{max}');
    grid on;

    % Donji red: usporedba upravljačkih signala
    subplot(2,3,4:6);
    hold on;
    colors = lines(length(results));
    for i = 1:length(results)
        plot(results{i}.U.Time, results{i}.U.Data, 'Color', colors(i,:), 'LineWidth', 1.5);
    end
    legend(labels, 'Location', 'best');
    xlabel('Vrijeme [s]'); ylabel('u [N]'); title('Usporedba upravljačkih signala');
    grid on;

    sgtitle('Test 3: Utjecaj ograničenja ulaza (u_{max})');
end

function plotTest4(QR_configs, metrics, results)
    figure('Name', 'Test 4: Težinske matrice Q, R', 'Position', [100 100 1200 800]);

    names = cellfun(@(c) c.name, QR_configs, 'UniformOutput', false);

    subplot(2,2,1);
    ISE_vals = cellfun(@(m) m.ISE_total, metrics);
    bar(categorical(names), ISE_vals);
    ylabel('ISE'); title('ISE');
    grid on;

    subplot(2,2,2);
    effort_vals = cellfun(@(m) m.TotalControlEffort, metrics);
    bar(categorical(names), effort_vals);
    ylabel('Napor'); title('Upravljački napor');
    grid on;

    subplot(2,2,3);
    hold on;
    colors = lines(length(results));
    for i = 1:length(results)
        plot(results{i}.X.Time, results{i}.X.Data(:,2), 'Color', colors(i,:), 'LineWidth', 1.5);
    end
    legend(names, 'Location', 'best');
    xlabel('Vrijeme [s]'); ylabel('\theta [rad]'); title('Kut njihala');
    grid on;

    subplot(2,2,4);
    hold on;
    for i = 1:length(results)
        plot(results{i}.U.Time, results{i}.U.Data, 'Color', colors(i,:), 'LineWidth', 1.5);
    end
    legend(names, 'Location', 'best');
    xlabel('Vrijeme [s]'); ylabel('u [N]'); title('Upravljački signal');
    grid on;

    sgtitle('Test 4: Utjecaj težinskih matrica (Q, R)');
end

function plotTest5(methods, metrics, results)
    figure('Name', 'Test 5: Metode integracije', 'Position', [100 100 1200 400]);

    subplot(1,3,1);
    ISE_vals = cellfun(@(m) m.ISE_total, metrics);
    bar(categorical(methods), ISE_vals);
    ylabel('ISE'); title('ISE vs Metoda');
    grid on;

    subplot(1,3,2);
    time_vals = cellfun(@(m) m.MeanSolveTime*1000, metrics);
    bar(categorical(methods), time_vals);
    ylabel('Vrijeme [ms]'); title('Vrijeme rješavanja vs Metoda');
    grid on;

    subplot(1,3,3);
    hold on;
    colors = lines(length(results));
    for i = 1:length(results)
        plot(results{i}.X.Time, results{i}.X.Data(:,2), 'Color', colors(i,:), 'LineWidth', 1.5);
    end
    legend(methods, 'Location', 'best');
    xlabel('Vrijeme [s]'); ylabel('\theta [rad]'); title('Usporedba trajektorija');
    grid on;

    sgtitle('Test 5: Usporedba metoda integracije');
end

function plotTest6(model_types, metrics, results)
    figure('Name', 'Test 6: Diskretni vs Kontinuirani', 'Position', [100 100 1200 400]);

    names = cellfun(@(c) c.name, model_types, 'UniformOutput', false);

    subplot(1,3,1);
    ISE_vals = cellfun(@(m) m.ISE_total, metrics);
    bar(categorical(names), ISE_vals);
    ylabel('ISE'); title('ISE');
    grid on;

    subplot(1,3,2);
    time_vals = cellfun(@(m) m.MeanSolveTime*1000, metrics);
    bar(categorical(names), time_vals);
    ylabel('Vrijeme [ms]'); title('Vrijeme rješavanja');
    grid on;

    subplot(1,3,3);
    hold on;
    colors = lines(length(results));
    for i = 1:length(results)
        plot(results{i}.X.Time, results{i}.X.Data(:,2), 'Color', colors(i,:), 'LineWidth', 1.5);
    end
    legend(names, 'Location', 'best');
    xlabel('Vrijeme [s]'); ylabel('\theta [rad]'); title('Usporedba trajektorija');
    grid on;

    sgtitle('Test 6: Diskretni vs Kontinuirani model');
end

function generateSummaryTable(all_results)
    %% Generiraj tablicu sažetka svih testova

    fprintf('\n');
    fprintf('╔══════════════════════════════════════════════════════════════════════════════╗\n');
    fprintf('║                           TABLICA SAŽETKA REZULTATA                          ║\n');
    fprintf('╠══════════════════════════════════════════════════════════════════════════════╣\n');

    % Test 1: Horizont N
    if isfield(all_results, 'test1')
        fprintf('║ TEST 1: Horizont predikcije (N)                                              ║\n');
        fprintf('╟──────────────────────────────────────────────────────────────────────────────╢\n');
        fprintf('║  N    │   ISE    │ IAE    │ Settling [s] │ Solve Time [ms] │ Iteracije     ║\n');
        fprintf('╟───────┼──────────┼────────┼──────────────┼─────────────────┼───────────────╢\n');
        for i = 1:length(all_results.test1.N_values)
            m = all_results.test1.metrics{i};
            fprintf('║  %3d  │ %7.4f  │ %6.3f │    %5.2f     │     %6.2f      │    %5.1f      ║\n', ...
                all_results.test1.N_values(i), m.ISE_total, m.IAE_total, ...
                m.SettlingTime_theta, m.MeanSolveTime*1000, m.MeanIterations);
        end
        fprintf('╠══════════════════════════════════════════════════════════════════════════════╣\n');
    end

    % Test 2: Ts
    if isfield(all_results, 'test2')
        fprintf('║ TEST 2: Vrijeme diskretizacije (Ts)                                          ║\n');
        fprintf('╟──────────────────────────────────────────────────────────────────────────────╢\n');
        fprintf('║  Ts [s] │   ISE    │ IAE    │ Settling [s] │ Ctrl Effort │ Final Error    ║\n');
        fprintf('╟─────────┼──────────┼────────┼──────────────┼─────────────┼────────────────╢\n');
        for i = 1:length(all_results.test2.Ts_values)
            m = all_results.test2.metrics{i};
            fprintf('║  %.3f   │ %7.4f  │ %6.3f │    %5.2f     │   %7.2f   │    %.6f   ║\n', ...
                all_results.test2.Ts_values(i), m.ISE_total, m.IAE_total, ...
                m.SettlingTime_theta, m.TotalControlEffort, m.FinalErrorNorm);
        end
        fprintf('╠══════════════════════════════════════════════════════════════════════════════╣\n');
    end

    fprintf('╚══════════════════════════════════════════════════════════════════════════════╝\n');

    % Spremi tablicu u datoteku
    fid = fopen('results/summary_table.txt', 'w');
    fprintf(fid, 'NMPC Test Results Summary\n');
    fprintf(fid, '=========================\n\n');

    if isfield(all_results, 'test1')
        fprintf(fid, 'Test 1: Prediction Horizon (N)\n');
        fprintf(fid, 'N\tISE\tIAE\tSettling[s]\tSolveTime[ms]\tIterations\n');
        for i = 1:length(all_results.test1.N_values)
            m = all_results.test1.metrics{i};
            fprintf(fid, '%d\t%.4f\t%.3f\t%.2f\t%.2f\t%.1f\n', ...
                all_results.test1.N_values(i), m.ISE_total, m.IAE_total, ...
                m.SettlingTime_theta, m.MeanSolveTime*1000, m.MeanIterations);
        end
        fprintf(fid, '\n');
    end

    fclose(fid);
    fprintf('\nTablica spremljena u: results/summary_table.txt\n');
end

%% ═══════════════════════════════════════════════════════════════════════
%  FUNKCIJE ZA PISANJE TABLICA PO TESTOVIMA
%% ═══════════════════════════════════════════════════════════════════════

function writeTestTable(folder, filename, headers, param_values, metrics, metric_fields, multipliers)
    %% Generička funkcija za pisanje tablice testa
    fid = fopen(fullfile(folder, filename), 'w');
    fprintf(fid, '%s\n', strjoin(headers, '\t'));
    fprintf(fid, '%s\n', repmat('-', 1, 60));

    for i = 1:length(param_values)
        m = metrics{i};
        fprintf(fid, '%g', param_values(i));
        for j = 1:length(metric_fields)
            val = m.(metric_fields{j});
            if j <= length(multipliers)
                val = val * multipliers(j);
            end
            fprintf(fid, '\t%.4f', val);
        end
        fprintf(fid, '\n');
    end
    fclose(fid);
end

function writeTestTable2(folder, filename, Ts_values, metrics)
    %% Tablica za Test 2: Vrijeme diskretizacije
    fid = fopen(fullfile(folder, filename), 'w');
    fprintf(fid, 'Ts[s]\tISE\tIAE\tSettling[s]\tCtrlEffort\tFinalError\n');
    fprintf(fid, '%s\n', repmat('-', 1, 60));

    for i = 1:length(Ts_values)
        m = metrics{i};
        fprintf(fid, '%.3f\t%.4f\t%.4f\t%.2f\t%.2f\t%.6f\n', ...
            Ts_values(i), m.ISE_total, m.IAE_total, m.SettlingTime_theta, ...
            m.TotalControlEffort, m.FinalErrorNorm);
    end
    fclose(fid);
end

function writeTestTable3(folder, filename, u_max_values, metrics)
    %% Tablica za Test 3: Ograničenje ulaza
    fid = fopen(fullfile(folder, filename), 'w');
    fprintf(fid, 'u_max[N]\tISE\tIAE\tSettling[s]\tMaxInput[N]\tCtrlEffort\n');
    fprintf(fid, '%s\n', repmat('-', 1, 60));

    for i = 1:length(u_max_values)
        m = metrics{i};
        fprintf(fid, '%d\t%.4f\t%.4f\t%.2f\t%.2f\t%.2f\n', ...
            u_max_values(i), m.ISE_total, m.IAE_total, m.SettlingTime_theta, ...
            m.MaxControlInput, m.TotalControlEffort);
    end
    fclose(fid);
end

function writeTestTable4(folder, filename, QR_configs, metrics)
    %% Tablica za Test 4: Težinske matrice
    fid = fopen(fullfile(folder, filename), 'w');
    fprintf(fid, 'Konfiguracija\tISE\tIAE\tSettling[s]\tCtrlEffort\tOvershoot[%%]\n');
    fprintf(fid, '%s\n', repmat('-', 1, 70));

    for i = 1:length(QR_configs)
        m = metrics{i};
        fprintf(fid, '%s\t%.4f\t%.4f\t%.2f\t%.2f\t%.2f\n', ...
            QR_configs{i}.name, m.ISE_total, m.IAE_total, m.SettlingTime_theta, ...
            m.TotalControlEffort, m.Overshoot_theta);
    end
    fclose(fid);
end

function writeTestTable5(folder, filename, methods, metrics)
    %% Tablica za Test 5: Metode integracije
    fid = fopen(fullfile(folder, filename), 'w');
    fprintf(fid, 'Metoda\tISE\tIAE\tSolveTime[ms]\tIterations\n');
    fprintf(fid, '%s\n', repmat('-', 1, 50));

    for i = 1:length(methods)
        m = metrics{i};
        fprintf(fid, '%s\t%.4f\t%.4f\t%.2f\t%.1f\n', ...
            methods{i}, m.ISE_total, m.IAE_total, m.MeanSolveTime*1000, m.MeanIterations);
    end
    fclose(fid);
end

function writeTestTable6(folder, filename, model_types, metrics)
    %% Tablica za Test 6: Diskretni vs Kontinuirani
    fid = fopen(fullfile(folder, filename), 'w');
    fprintf(fid, 'Model\tISE\tIAE\tSolveTime[ms]\tIterations\tSettling[s]\n');
    fprintf(fid, '%s\n', repmat('-', 1, 60));

    for i = 1:length(model_types)
        m = metrics{i};
        fprintf(fid, '%s\t%.4f\t%.4f\t%.2f\t%.1f\t%.2f\n', ...
            model_types{i}.name, m.ISE_total, m.IAE_total, m.MeanSolveTime*1000, ...
            m.MeanIterations, m.SettlingTime_theta);
    end
    fclose(fid);
end
