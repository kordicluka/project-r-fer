classdef SimulationEnvironment < handle
    %% SimulationEnvironment - Upravljanje simulacijom NMPC sustava
    %
    % Ova klasa upravlja closed-loop simulacijom cart-pendulum sustava.
    % Podržava različite vrste poremećaja i dinamičku promjenu parametara modela.
    %
    % Poremećaji:
    %   - disturbance_force: Sila na kolica [N] (dodaje se na upravljački signal)
    %   - disturbance_torque: Moment na njihalo [Nm] (djeluje direktno na kutnu akceleraciju)
    %   - param_change: Funkcija za promjenu parametara modela tijekom simulacije
    %
    % Primjer korištenja:
    %   sim = SimulationEnvironment(model, controller, T_end, x0, x_target);
    %   sim.disturbance_force = @(t) 10 * (t > 5 & t < 5.5);
    %   sim.disturbance_torque = @(t) 2 * (t > 3 & t < 3.2);
    %   sim.param_change = @(t, model) changeParams(t, model);
    %   sim.run();

    properties
        % Simulacijski parametri
        T_end           % ukupno vrijeme simulacije [s]
        x0              % početno stanje [4x1]
        x_target        % ciljno stanje (referenca) [4x1]

        % Poremećaji
        disturbance_force   % @(t) -> sila na kolica [N]
        disturbance_torque  % @(t) -> moment na njihalo [Nm]
        param_change        % @(t, model) -> promjena parametara modela

        % Reference na druge klase
        model           % CartPendulumModel objekt
        controller      % NMPCController objekt

        % Spremanje rezultata
        results
    end

    properties (Dependent)
        % Ts i integration_method se čitaju iz kontrolera
        Ts
        integration_method
    end

    methods
        %% Dependent property getters
        function val = get.Ts(obj)
            val = obj.controller.Ts;
        end

        function val = get.integration_method(obj)
            val = obj.controller.integration_method;
        end

        %% Konstruktor
        function obj = SimulationEnvironment(model, controller, T_end, x0, x_target, varargin)
            %% Constructor - Inicijalizacija simulacijskog okruženja
            %
            % Argumenti:
            %   model      - CartPendulumModel objekt
            %   controller - NMPCController objekt
            %   T_end      - Trajanje simulacije [s]
            %   x0         - Početno stanje [4x1]
            %   x_target   - Ciljno stanje [4x1]
            %
            % Opcionalni name-value parovi:
            %   'disturbance_force'  - @(t) sila na kolica [N]
            %   'disturbance_torque' - @(t) moment na njihalo [Nm]
            %   'param_change'       - @(t, model) funkcija za promjenu parametara

            obj.model = model;
            obj.controller = controller;
            obj.T_end = T_end;
            obj.x0 = x0;
            obj.x_target = x_target;

            % Parse optional arguments
            p = inputParser;
            addParameter(p, 'disturbance_force', @(t) 0);
            addParameter(p, 'disturbance_torque', @(t) 0);
            addParameter(p, 'param_change', @(t, model) []);
            parse(p, varargin{:});

            obj.disturbance_force = p.Results.disturbance_force;
            obj.disturbance_torque = p.Results.disturbance_torque;
            obj.param_change = p.Results.param_change;

            % Inicijalizacija strukture za rezultate
            obj.results = struct('X', [], 'U', [], 'Cost', [], ...
                'SolveTime', [], 'Iterations', [], 'Metrics', []);
        end

        %% Glavna simulacijska petlja
        function run(obj)
            %% run - Pokreni closed-loop simulaciju
            %
            % Izvršava NMPC kontrolnu petlju:
            %   1. Izračunaj optimalni upravljački signal
            %   2. Primijeni poremećaje (sila, moment, promjena parametara)
            %   3. Simuliraj jedan korak sustava
            %   4. Ponovi

            % Dohvati Ts iz kontrolera
            Ts = obj.controller.Ts;
            integration_method = obj.controller.integration_method;

            % Inicijalizacija
            x = obj.x0;

            % Priprema polja za spremanje rezultata
            time_vec = (0:Ts:obj.T_end)';
            nSteps = length(time_vec);
            X_vec = zeros(nSteps, length(x));
            U_vec = zeros(nSteps-1, obj.model.nu);
            Cost_vec = zeros(nSteps-1, 1);
            SolveTime_vec = zeros(nSteps-1, 1);
            Iterations_vec = zeros(nSteps-1, 1);

            X_vec(1, :) = x';

            % Pripremi referentnu trajektoriju (konstantna)
            yref = obj.x_target';  % [1 x ny] vektor

            % Spremi originalne parametre modela (za restore ako treba)
            original_params = obj.model.getAllParams();

            % Glavna petlja
            h = waitbar(0, 'Simulacija u tijeku...');
            for i = 1:nSteps-1
                t = time_vec(i);

                % 1. Primijeni promjenu parametara modela (ako je definirana)
                obj.param_change(t, obj.model);

                % 2. Izračun optimalnog upravljačkog signala
                [u, cost, opt_info] = obj.controller.computeControlAction(x, yref);

                % 3. Dodaj poremećaj sile na kolica
                d_force = obj.disturbance_force(t);
                u_total = u + d_force;

                % 4. Dohvati poremećaj momenta na njihalo
                d_torque = obj.disturbance_torque(t);

                % 5. Simulacija sustava s poremećajem momenta
                x_next = obj.simulateStepWithTorque(x, u_total, Ts, integration_method, d_torque);

                % 6. Ažuriranje stanja
                x = x_next;

                % 7. Spremi rezultate
                X_vec(i+1, :) = x';
                U_vec(i, :) = u_total';
                Cost_vec(i) = cost;
                SolveTime_vec(i) = opt_info.SolveTime;
                Iterations_vec(i) = opt_info.Iterations;

                % Ažuriranje waitbara
                waitbar(time_vec(i+1) / obj.T_end, h);
            end
            close(h);

            % Vrati originalne parametre (opcionalno)
            % obj.restoreParams(original_params);

            % Spremanje rezultata kao timeseries objekte
            obj.results.X = timeseries(X_vec, time_vec, 'Name', 'Stanja');
            obj.results.U = timeseries(U_vec, time_vec(1:end-1), 'Name', 'Ulaz');
            obj.results.Cost = timeseries(Cost_vec, time_vec(1:end-1), 'Name', 'Cijena');
            obj.results.SolveTime = timeseries(SolveTime_vec, time_vec(1:end-1), 'Name', 'VrijemeRjesavanja');
            obj.results.Iterations = timeseries(Iterations_vec, time_vec(1:end-1), 'Name', 'BrojIteracija');

            % Izračunaj metrike performansi
            obj.results.Metrics = obj.calculateMetrics();
        end

        %% Simulacija s poremećajem momenta
        function x_next = simulateStepWithTorque(obj, x, u, Ts, method, torque)
            %% simulateStepWithTorque - Simulira korak s dodatnim momentom na njihalo
            %
            % Poremećaj momenta djeluje direktno na kutnu akceleraciju (omega_dot).
            % Modificira se stanje nakon standardne integracije.
            %
            % Argumenti:
            %   x      - trenutno stanje [4x1]
            %   u      - upravljački signal (sila na kolica)
            %   Ts     - vremenski korak
            %   method - metoda integracije
            %   torque - vanjski moment na njihalo [Nm]

            % Standardna simulacija
            x_next = obj.model.simulateStep(x, u, Ts, method);

            % Dodaj efekt vanjskog momenta na kutnu brzinu
            % torque = I * alpha => alpha = torque / I
            % Za točkastu masu na kraju štapa: I = m * l^2
            if torque ~= 0
                m = obj.model.params.m;
                l = obj.model.params.l;
                I = m * l^2;  % Moment inercije
                alpha = torque / I;  % Kutna akceleracija

                % omega_next = omega + alpha * Ts
                x_next(4) = x_next(4) + alpha * Ts;
            end
        end

        %% Pomoćne metode za promjenu parametara modela
        function setModelParam(obj, paramName, value)
            %% setModelParam - Postavi parametar modela
            obj.model.setParams(paramName, value);
        end

        function value = getModelParam(obj, paramName)
            %% getModelParam - Dohvati parametar modela
            value = obj.model.getParam(paramName);
        end

        function params = getModelParams(obj)
            %% getModelParams - Dohvati sve parametre modela
            params = obj.model.getAllParams();
        end

        %% Metrike performansi
        function metrics = calculateMetrics(obj)
            %% calculateMetrics - Izračunaj metrike performansi kontrolera
            %
            % Metrike:
            %   - ISE: Integral Square Error (suma kvadrata greške)
            %   - IAE: Integral Absolute Error (suma apsolutne greške)
            %   - ITAE: Integral Time-weighted Absolute Error
            %   - SettlingTime: Vrijeme smirivanja (2% kriterij)
            %   - Overshoot: Maksimalno prekoračenje [%]
            %   - TotalControlEffort: Ukupni upravljački napor
            %   - MeanSolveTime: Prosječno vrijeme rješavanja optimizacije
            %   - MaxSolveTime: Maksimalno vrijeme rješavanja
            %   - MeanIterations: Prosječan broj iteracija
            %   - MaxIterations: Maksimalan broj iteracija

            X = obj.results.X.Data;
            U = obj.results.U.Data;
            T = obj.results.X.Time;
            Ts = obj.controller.Ts;

            % Greška stanja (odstupanje od cilja)
            error = X - repmat(obj.x_target', size(X, 1), 1);

            % ISE - Integral Square Error (za svako stanje i ukupno)
            metrics.ISE_states = Ts * sum(error.^2, 1);  % [1x4] za svako stanje
            metrics.ISE_total = sum(metrics.ISE_states);

            % IAE - Integral Absolute Error
            metrics.IAE_states = Ts * sum(abs(error), 1);
            metrics.IAE_total = sum(metrics.IAE_states);

            % ITAE - Integral Time-weighted Absolute Error
            time_weights = T;
            metrics.ITAE_states = Ts * sum(time_weights .* abs(error), 1);
            metrics.ITAE_total = sum(metrics.ITAE_states);

            % Settling time (2% kriterij) - za kut theta (stanje 2)
            theta_error = abs(error(:, 2));
            theta_target = abs(obj.x_target(2));
            if theta_target == 0
                threshold = 0.02;  % 2% od 1 rad ako je cilj 0
            else
                threshold = 0.02 * theta_target;
            end

            % Nađi zadnje vrijeme kada je greška bila iznad praga
            settled_idx = find(theta_error > threshold, 1, 'last');
            if isempty(settled_idx)
                metrics.SettlingTime_theta = 0;
            elseif settled_idx >= length(T)
                metrics.SettlingTime_theta = Inf;  % Nije se smirio
            else
                metrics.SettlingTime_theta = T(settled_idx + 1);
            end

            % Settling time za poziciju (stanje 1)
            pos_error = abs(error(:, 1));
            pos_threshold = 0.02;  % 2cm
            settled_idx_pos = find(pos_error > pos_threshold, 1, 'last');
            if isempty(settled_idx_pos)
                metrics.SettlingTime_position = 0;
            elseif settled_idx_pos >= length(T)
                metrics.SettlingTime_position = Inf;
            else
                metrics.SettlingTime_position = T(settled_idx_pos + 1);
            end

            % Overshoot za kut (ako cilj je 0)
            if obj.x_target(2) == 0
                max_theta = max(abs(X(:, 2)));
                initial_theta = abs(obj.x0(2));
                if initial_theta > 0
                    metrics.Overshoot_theta = 100 * (max_theta - initial_theta) / initial_theta;
                else
                    metrics.Overshoot_theta = 0;
                end
            else
                metrics.Overshoot_theta = NaN;
            end

            % Ukupni upravljački napor
            metrics.TotalControlEffort = Ts * sum(abs(U));
            metrics.MaxControlInput = max(abs(U));

            % Optimizacijske metrike
            solve_times = obj.results.SolveTime.Data;
            iterations = obj.results.Iterations.Data;

            metrics.MeanSolveTime = mean(solve_times);
            metrics.MaxSolveTime = max(solve_times);
            metrics.MinSolveTime = min(solve_times);
            metrics.TotalSolveTime = sum(solve_times);

            metrics.MeanIterations = mean(iterations);
            metrics.MaxIterations = max(iterations);
            metrics.MinIterations = min(iterations);

            % Završno stanje
            metrics.FinalState = X(end, :)';
            metrics.FinalError = error(end, :)';
            metrics.FinalErrorNorm = norm(metrics.FinalError);
        end

        function printMetrics(obj)
            %% printMetrics - Ispiši metrike performansi
            m = obj.results.Metrics;

            fprintf('\n========== METRIKE PERFORMANSI ==========\n\n');

            fprintf('--- Greška praćenja ---\n');
            fprintf('ISE (ukupno):     %.4f\n', m.ISE_total);
            fprintf('IAE (ukupno):     %.4f\n', m.IAE_total);
            fprintf('ITAE (ukupno):    %.4f\n', m.ITAE_total);
            fprintf('ISE po stanjima:  [%.4f, %.4f, %.4f, %.4f]\n', m.ISE_states);
            fprintf('\n');

            fprintf('--- Vrijeme smirivanja (2%% kriterij) ---\n');
            fprintf('Settling time (θ):  %.3f s\n', m.SettlingTime_theta);
            fprintf('Settling time (x):  %.3f s\n', m.SettlingTime_position);
            fprintf('\n');

            fprintf('--- Upravljanje ---\n');
            fprintf('Ukupni napor:     %.4f\n', m.TotalControlEffort);
            fprintf('Max |u|:          %.4f N\n', m.MaxControlInput);
            fprintf('\n');

            fprintf('--- Optimizacija ---\n');
            fprintf('Prosj. vrijeme:   %.4f ms\n', m.MeanSolveTime * 1000);
            fprintf('Max vrijeme:      %.4f ms\n', m.MaxSolveTime * 1000);
            fprintf('Ukupno vrijeme:   %.4f s\n', m.TotalSolveTime);
            fprintf('Prosj. iteracije: %.1f\n', m.MeanIterations);
            fprintf('Max iteracije:    %d\n', m.MaxIterations);
            fprintf('\n');

            fprintf('--- Završno stanje ---\n');
            fprintf('x_final:          [%.4f, %.4f, %.4f, %.4f]\n', m.FinalState);
            fprintf('Greška (norma):   %.6f\n', m.FinalErrorNorm);

            fprintf('\n==========================================\n');
        end

        %% Animacija
        function animate(obj)
            %% animate - Vizualizacija simulacije

            % Ekstrakcija podataka iz timeseries objekata
            X = obj.results.X.Data;
            T = obj.results.X.Time;

            % Položaj kolica (x) i kut njihala (theta)
            px = X(:, 1);      % Položaj kolica (x-os)
            theta = X(:, 2);   % Kut njihala

            % Parametri
            l = obj.model.params.l; % Duljina njihala
            cart_width = 0.3;       % Širina kolica
            cart_height = 0.15;     % Visina kolica

            % Postavljanje koordinatnog sustava
            figure;
            hold on;
            axis equal;
            grid on;
            xlim([-2 2]);
            ylim([-1.2 1.2]);
            xlabel('Položaj kolica [m]');
            ylabel('Visina [m]');
            title('Simulacija njihala na kolicima');

            % Inicijalni crtež
            cart = rectangle('Position',[px(1)-cart_width/2, -cart_height/2, cart_width, cart_height], ...
                             'FaceColor',[0.1 0.4 0.8]);

            pendulum = line([px(1), px(1)+l*sin(theta(1))], ...
                            [0, l*cos(theta(1))], 'Color','r','LineWidth',2);

            % Animacija
            for k = 1:10:length(T)
                % Ažuriraj položaj
                set(cart, 'Position', [px(k)-cart_width/2, -cart_height/2, cart_width, cart_height]);
                set(pendulum, 'XData', [px(k), px(k)+l*sin(theta(k))]);
                set(pendulum, 'YData', [0, l*cos(theta(k))]);
                drawnow;
            end
        end

    end
end
