classdef NMPCController
    properties
        model
        N % Prediction horizon
        Q % State cost matrix
        R % Input cost matrix
        u_max % Input constraint
        Ts % Sample time
        x_target % Target state
    end

    methods
        function obj = NMPCController(model, N, Q, R, u_max, Ts, x_target)
            obj.model = model;
            obj.N = N;
            obj.Q = Q;
            obj.R = R;
            obj.u_max = u_max;
            obj.Ts = Ts;
            obj.x_target = x_target;
        end

        function u_opt = computeControlAction(obj, x0)
            % Define optimization variables (sequence of control inputs)
            u_seq0 = zeros(obj.N, 1);

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
            [u_seq_opt, ~] = fmincon(cost_fcn, u_seq0, A, b, Aeq, beq, lb, ub, nonlcon, options);

            % Return the first control action
            u_opt = u_seq_opt(1);
        end

        function cost = predictionCost(obj, x0, u_seq)
            cost = 0;
            x_seq = zeros(length(x0), obj.N + 1);
            x_seq(:, 1) = x0;

            for k = 1:obj.N
                % Predict next state using the model
                x_seq(:, k+1) = obj.model.simulateStep(x_seq(:, k), u_seq(k), obj.Ts);
                
                % Calculate error from target state
                error = x_seq(:, k+1) - obj.x_target;

                error(2) = atan2(sin(error(2)), cos(error(2)));
                
                % Add to cost (tracking cost)
                cost = cost + error' * obj.Q * error + u_seq(k)' * obj.R * u_seq(k);
            end
        end
    end
end