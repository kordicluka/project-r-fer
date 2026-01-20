classdef NMPCController < handle
    %% NMPCController - Nonlinear Model Predictive Controller using MATLAB's nlmpc
    %
    % This controller uses MATLAB's Model Predictive Control Toolbox (nlmpc)
    % to solve the nonlinear optimal control problem for the cart-pendulum system.
    %
    % References:
    % - https://www.mathworks.com/help/mpc/ref/nlmpc.html
    % - https://www.mathworks.com/help/mpc/ug/swing-up-control-of-a-pendulum-using-nonlinear-model-predictive-control.html

    properties
        model               % CartPendulumModel object
        nlobj               % nlmpc object from MPC Toolbox
        N                   % Prediction horizon
        Q                   % State cost matrix
        R                   % Input cost matrix
        u_max               % Input constraint
        Ts                  % Sample time
        integration_method  % Integration method for simulation (only used when use_continuous_model=false)
        use_continuous_model % If true, use continuous-time model (nlmpc does discretization)

        % For tracking previous control action
        u_prev

        % For nlmpcmove options
        nloptions
    end

    methods
        function obj = NMPCController(model, N, Q, R, u_max, Ts, integration_method, use_continuous_model)
            %% Constructor - Initialize NMPC controller with nlmpc object
            %
            % Arguments:
            %   model       - CartPendulumModel object
            %   N           - Prediction horizon
            %   Q           - State cost matrix (diagonal)
            %   R           - Input cost weight
            %   u_max       - Maximum control input
            %   Ts          - Sample time
            %   integration_method - 'rk4', 'euler', or 'ode45' (used only if use_continuous_model=false)
            %   use_continuous_model - If true, nlmpc uses continuous model and discretizes internally

            obj.model = model;
            obj.N = N;
            obj.Q = Q;
            obj.R = R;
            obj.u_max = u_max;
            obj.Ts = Ts;
            obj.integration_method = integration_method;
            obj.use_continuous_model = use_continuous_model;

            % Initialize previous control action
            obj.u_prev = 0;

            % Create the nlmpc object
            obj.createNLMPC();

            % Initialize nlmpcmove options
            obj.nloptions = nlmpcmoveopt;
        end

        function createNLMPC(obj)
            %% Create and configure the nlmpc object

            % Create nlmpc object: nx states, ny outputs, nu inputs
            nx = obj.model.nx;
            ny = obj.model.ny;
            nu = obj.model.nu;
            obj.nlobj = nlmpc(nx, ny, nu);

            % Set sample time and horizons
            obj.nlobj.Ts = obj.Ts;
            obj.nlobj.PredictionHorizon = obj.N;
            obj.nlobj.ControlHorizon = obj.N;

            % Define the prediction model
            obj.nlobj.Model.NumberOfParameters = 0;

            if obj.use_continuous_model
                % CONTINUOUS-TIME formulation: nlmpc discretizes internally
                % StateFcn returns dx/dt (state derivatives)
                obj.nlobj.Model.StateFcn = @(x, u) obj.model.stateFcn(x, u);
                obj.nlobj.Model.IsContinuousTime = true;

                % Provide analytical Jacobian for state function (improves optimization)
                obj.nlobj.Jacobian.StateFcn = @(x, u) obj.nlmpcStateJacobian(x, u);
            else
                % DISCRETE-TIME formulation: we do integration manually
                % StateFcn returns x(k+1) (next state)
                obj.nlobj.Model.StateFcn = @(x, u) obj.nlmpcStateFcn(x, u);
                obj.nlobj.Model.IsContinuousTime = false;
            end

            % Define output function using model's outputFcn
            obj.nlobj.Model.OutputFcn = @(x, u) obj.nlmpcOutputFcn(x, u);

            % Provide analytical Jacobian for output function (improves speed)
            obj.nlobj.Jacobian.OutputFcn = @(x, u) obj.nlmpcOutputJacobian(x, u);

            % Configure cost function weights
            % Extract diagonal elements from Q matrix for output weights
            Q_diag = diag(obj.Q);
            obj.nlobj.Weights.OutputVariables = Q_diag';
            obj.nlobj.Weights.ManipulatedVariables = obj.R;

            % Set input constraints
            obj.nlobj.MV.Min = -obj.u_max;
            obj.nlobj.MV.Max = obj.u_max;

            % Validate the functions (optional but recommended)
            x0 = zeros(obj.model.nx, 1);
            u0 = 0;
            try
                validateFcns(obj.nlobj, x0, u0);
                fprintf('NMPC functions validated successfully.\n');
            catch ME
                warning(ME.identifier, 'NMPC function validation failed: %s', ME.message);
            end
        end

        function [u_opt, cost, opt_info] = computeControlAction(obj, x0, yref)
            %% Compute optimal control action using nlmpcmove
            %
            % Inputs:
            %   x0   - current state [4x1]
            %   yref - reference trajectory [(N+1) x ny] or [1 x ny] for constant reference
            %
            % Outputs:
            %   u_opt    - optimal control action (scalar)
            %   cost     - cost value from optimization
            %   opt_info - structure with optimization info:
            %              .SolveTime  - time to solve [s]
            %              .Iterations - number of iterations
            %              .ExitFlag   - solver exit flag

            % If yref is a single row, replicate it for the entire horizon
            if size(yref, 1) == 1
                yref = repmat(yref, obj.N+1, 1);
            end

            % Solve the NMPC optimization problem with timing
            tic;
            [u_opt, obj.nloptions, info] = nlmpcmove(obj.nlobj, x0, obj.u_prev, yref, [], obj.nloptions);
            solve_time = toc;

            % Extract cost from info structure
            cost = info.Cost;

            % Build optimization info structure
            opt_info.SolveTime = solve_time;
            opt_info.Iterations = info.Iterations;
            opt_info.ExitFlag = info.ExitFlag;

            % Update previous control action for next iteration
            obj.u_prev = u_opt;

            % Display optimization status if failed
            if info.ExitFlag <= 0
                warning('NMPC optimization did not converge properly. ExitFlag: %d', info.ExitFlag);
            end
        end

        function x_next = nlmpcStateFcn(obj, x, u)
            %% Discrete-time state function for nlmpc
            %
            % This function is called by nlmpc to predict future states.
            % It uses the model's simulateStep method with the specified integration method.
            %
            % Inputs:
            %   x - current state [4x1]
            %   u - control input (scalar)
            %
            % Outputs:
            %   x_next - next state [4x1]

            % Use the model's simulateStep for prediction
            x_next = obj.model.simulateStep(x, u, obj.Ts, obj.integration_method);
        end

        function [A, B] = nlmpcStateJacobian(obj, x, u)
            %% Analytical Jacobian of continuous-time state function
            %
            % This is used when use_continuous_model=true.
            % Returns Jacobians of dx/dt with respect to x and u.
            %
            % Inputs:
            %   x - current state [4x1]
            %   u - control input (scalar)
            %
            % Outputs:
            %   A - Jacobian df/dx [nx x nx]
            %   B - Jacobian df/du [nx x nu]

            [A, B] = obj.model.stateFcnJacobian(x, u);
        end

        function y = nlmpcOutputFcn(obj, x, u)
            %% Output function for nlmpc
            %
            % Maps states to outputs. For tracking, we output all states.
            %
            % Inputs:
            %   x - current state [4x1]
            %   u - control input (scalar)
            %
            % Outputs:
            %   y - output vector [4x1]

            % Use the model's outputFcn (identity mapping: y = x)
            y = obj.model.outputFcn(x, u);
        end

        function C = nlmpcOutputJacobian(obj, x, u)
            %% Analytical Jacobian of output function with respect to state
            %
            % Providing analytical Jacobian improves computational efficiency.
            %
            % Inputs:
            %   x - current state [4x1]
            %   u - control input (scalar)
            %
            % Outputs:
            %   C - Jacobian matrix [ny x nx]

            % Get Jacobian from model (identity: C = I, D = 0)
            [C, ~] = obj.model.outputFcnJacobian(x, u);
        end
    end
end
