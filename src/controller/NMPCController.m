classdef NMPCController
    properties
        model
        N % Prediction horizon
        Q % State cost matrix
        R % Input cost matrix
        u_max % Input constraint
        Ts % Sample time
        integration_method % Integration method for simulation
        
        data
    end

    methods
        function obj = NMPCController(model, N, Q, R, u_max, Ts, x_target, integration_method)
            obj.model = model;
            obj.N = N;
            obj.Q = Q;
            obj.R = R;
            obj.u_max = u_max;
            obj.Ts = Ts;
            obj.x_target = x_target;
            obj.integration_method = integration_method;
            
            % Initialize previous optimal sequence for warm start
            obj.u_seq_opt_prev = zeros(N, 1);
        end

        function [u_opt, cost] = computeControlAction(obj, x0, x_target)
            % Use the previous solution as an initial guess (warm start)
            u_seq0 = obj.u_seq_opt_prev;

            % Define cost function
            cost_fcn = @(u_seq) obj.predictionCost(x0, u_seq);

            % Define constraints
            A = [];
            b = [];
            Aeq = [];
            beq = [];
            lb = -obj.u_max * ones(obj.N, 1);
            ub = obj.u_max * ones(obj.N, 1);
            
            % No nonlinear constraints in this simple formulation
            nonlcon = [];
            
            % Optimization options
            options = optimoptions('fmincon', 'Display', 'none', 'Algorithm', 'sqp');

            % Solve the optimization problem
            [u_seq_opt, cost] = fmincon(cost_fcn, u_seq0, A, b, Aeq, beq, lb, ub, nonlcon, options);

            % Prepare for next iteration's warm start (shift solution)
            obj.u_seq_opt_prev = [u_seq_opt(2:end); 0];

            % Return the first control action
            u_opt = u_seq_opt(1);
        end

        function cost = predictionCost(obj, x0, u_seq)
            cost = 0;
            x_seq = zeros(length(x0), obj.N + 1);
            x_seq(:, 1) = x0;

            for k = 1:obj.N
                % Predict next state using the model
                x_seq(:, k+1) = obj.model.simulateStep(x_seq(:, k), u_seq(k), obj.Ts, obj.integration_method);
                
                % Calculate error from target state
                error = x_seq(:, k+1) - obj.x_target;

                error(2) = atan2(sin(error(2)), cos(error(2)));
                
                % Add to cost (tracking cost)
                cost = cost + error' * obj.Q * error + u_seq(k)' * obj.R * u_seq(k);
            end
        end
    end
end