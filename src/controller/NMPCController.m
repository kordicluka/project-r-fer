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
        x_target            % Target state
        integration_method  % Integration method for simulation

        % For tracking previous control action
        u_prev

        % For nlmpcmove options
        nloptions
    end

    methods
        function obj = NMPCController(model, N, Q, R, u_max, Ts, x_target, integration_method)
            %% Constructor - Initialize NMPC controller with nlmpc object
            obj.model = model;
            obj.N = N;
            obj.Q = Q;
            obj.R = R;
            obj.u_max = u_max;
            obj.Ts = Ts;
            obj.x_target = x_target;
            obj.integration_method = integration_method;

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

            % Define the prediction model using discrete-time state function
            % Use anonymous function to properly wrap the method call
            obj.nlobj.Model.StateFcn = @(x, u) obj.nlmpcStateFcn(x, u);
            obj.nlobj.Model.IsContinuousTime = false;
            obj.nlobj.Model.NumberOfParameters = 0; % No additional parameters needed

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
            x0 = obj.x_target;
            u0 = 0;
            try
                validateFcns(obj.nlobj, x0, u0);
                fprintf('NMPC functions validated successfully.\n');
            catch ME
                warning(ME.identifier, 'NMPC function validation failed: %s', ME.message);
            end
        end

        function [u_opt, cost] = computeControlAction(obj, x0)
            %% Compute optimal control action using nlmpcmove
            %
            % Inputs:
            %   x0 - current state [4x1]
            %
            % Outputs:
            %   u_opt - optimal control action (scalar)
            %   cost  - cost value from optimization

            % Reference trajectory (constant target state)
            yref = repmat(obj.x_target', obj.N+1, 1);

            % Solve the NMPC optimization problem
            [u_opt, obj.nloptions, info] = nlmpcmove(obj.nlobj, x0, obj.u_prev, yref, [], obj.nloptions);

            % Extract cost from info structure
            cost = info.Cost;

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

            % Use the model's outputFcn
            y = obj.model.outputFcn(x, u);

            % Apply angle wrapping for the pendulum angle (second state)
            % This ensures proper handling of the periodic nature of theta
            y(2) = atan2(sin(x(2)), cos(x(2)));
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

            % Get Jacobian from model
            [C, ~] = obj.model.outputFcnJacobian(x, u);

            % Modify second row for angle wrapping: d(atan2(sin(theta), cos(theta)))/dtheta
            % For small angles, this is approximately 1, but let's be precise:
            theta = x(2);
            % Derivative of atan2(sin(theta), cos(theta)) with respect to theta is 1
            % (since atan2(sin(theta), cos(theta)) = theta for theta in [-pi, pi])
            % The Jacobian for the wrapped angle is the same as identity
            C(2, :) = [0, 1, 0, 0];
        end
    end
end
