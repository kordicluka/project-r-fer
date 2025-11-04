classdef SimulationEnvironment < handle
    properties
        % Simulacijski parametri
        Ts              % vremenski korak
        T_end           % ukupno vrijeme
        x0              % početno stanje
        disturbance     % poremećaji

        % Reference na druge klase
        model           % objekt System_Model
        controller      % objekt NMPC_Controller

        % Spremanje rezultata
        results         % struktura s poljima T, X, U
    end

    methods
        % Konstruktor
        function obj = SimulationEnvironment(model, controller, Ts, T_end, x0, disturbance)
            obj.model = model;
            obj.controller = controller;
            obj.Ts = Ts;
            obj.T_end = T_end;
            obj.x0 = x0;

            if nargin < 6 || isempty(disturbance)
                obj.disturbance = @(t) 0; % zadano bez poremećaja
            else
                obj.disturbance = disturbance;
            end

            obj.results = struct('T', [], 'X', [], 'U', []);
        end

        function run(obj)
            % Inicijalizacija
            t = 0;
            x = obj.x0;
            Tstep = obj.Ts;
        
            % Spremanje početnih vrijednosti
            obj.results.T = t;
            obj.results.X = x';
            obj.results.U = [];
        
            % Glavna petlja
            h = waitbar(0, 'Simulacija u tijeku...');
            while t < obj.T_end
                % Izračun ulaza pomoću kontrolera
                u = obj.controller.computeControlAction(x);
        
                % Dodavanje poremećaja
                d = obj.disturbance(t);
                u = u + d;
        
                % Simulacija sustava preko modela
                x_next = obj.model.simulateStep(x, u, Tstep);
        
                % Ažuriranje vremena i stanja
                t = t + Tstep;
                x = x_next;
        
                % Spremi rezultate
                obj.results.T = [obj.results.T; t];
                obj.results.X = [obj.results.X; x'];
                obj.results.U = [obj.results.U; u];

                % Ažuriranje waitbara
                waitbar(t / obj.T_end, h);
            end
            close(h);
        end

        function animate(obj)
            X = obj.results.X;
            T = obj.results.T;
        
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
            
            % --- ISPRAVAK OVDJE ---
            pendulum = line([px(1), px(1)+l*sin(theta(1))], ...
                            [0, l*cos(theta(1))], 'Color','r','LineWidth',2);
        
            % Animacija
            for k = 1:10:length(T)
                % Ažuriraj položaj
                set(cart, 'Position', [px(k)-cart_width/2, -cart_height/2, cart_width, cart_height]);
                set(pendulum, 'XData', [px(k), px(k)+l*sin(theta(k))]);
                
                % --- ISPRAVAK OVDJE ---
                set(pendulum, 'YData', [0, l*cos(theta(k))]);
                
                drawnow;
            end
        end

    end
end