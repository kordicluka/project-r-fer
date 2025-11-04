classdef SimulationEnvironment < handle
    properties
        % Simulacijski parametri
        Ts              % vremenski korak
        T_end           % ukupno vrijeme
        x0              % početno stanje
        disturbance     % poremećaji
        integration_method % metoda integracije

        % Reference na druge klase
        model           % objekt System_Model
        controller      % objekt NMPC_Controller

        % Spremanje rezultata
        results
    end

    methods
        % Konstruktor
        function obj = SimulationEnvironment(model, controller, Ts, T_end, x0, disturbance, integration_method)
            obj.model = model;
            obj.controller = controller;
            obj.Ts = Ts;
            obj.T_end = T_end;
            obj.x0 = x0;
            obj.integration_method = integration_method;

            if nargin < 6 || isempty(disturbance)
                obj.disturbance = @(t) 0; % zadano bez poremećaja
            else
                obj.disturbance = disturbance;
            end

            % Inicijalizacija strukture za rezultate
            obj.results = struct('X', [], 'U', [], 'Cost', []);
        end

        function run(obj)
            % Inicijalizacija
            x = obj.x0;
            Tstep = obj.Ts;
            
            % Priprema polja za spremanje rezultata
            time_vec = (0:Tstep:obj.T_end)';
            X_vec = zeros(length(time_vec), length(x));
            U_vec = zeros(length(time_vec)-1, obj.model.nu);
            Cost_vec = zeros(length(time_vec)-1, 1);
            
            X_vec(1, :) = x';

            % Glavna petlja
            h = waitbar(0, 'Simulacija u tijeku...');
            for i = 1:length(time_vec)-1
                % Izračun ulaza pomoću kontrolera
                [u, cost] = obj.controller.computeControlAction(x);
        
                % Dodavanje poremećaja
                d = obj.disturbance(time_vec(i));
                u = u + d;
        
                % Simulacija sustava preko modela
                x_next = obj.model.simulateStep(x, u, Tstep, obj.integration_method);
        
                % Ažuriranje stanja
                x = x_next;
        
                % Spremi rezultate
                X_vec(i+1, :) = x';
                U_vec(i, :) = u';
                Cost_vec(i) = cost;

                % Ažuriranje waitbara
                waitbar(time_vec(i+1) / obj.T_end, h);
            end
            close(h);
            
            % Spremanje rezultata kao timeseries objekte
            obj.results.X = timeseries(X_vec, time_vec, 'Name', 'Stanja');
            obj.results.U = timeseries(U_vec, time_vec(1:end-1), 'Name', 'Ulaz');
            obj.results.Cost = timeseries(Cost_vec, time_vec(1:end-1), 'Name', 'Cijena');
        end

        function animate(obj)
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
            ylim([-1.2 1.2]); % Ostavljamo isti Y-limit, daje dobar pregled
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