classdef SimulationEnvironment
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
    end

end
