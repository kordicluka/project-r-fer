classdef CartPendulumModel < handle

    properties
        nx = 4;
        nu = 1;
        ny = 4;

        params % pendulum parameters
    end

    methods
        function obj = CartPendulumModel()
            obj.params = obj.populateParams();
        end

        function dx = stateFcn(obj, x, u)
            %% stateFcn - Compute state derivatives for cart-pendulum system
            %
            % Inputs:
            %   x - state vector [px; theta; vx; omega] (4x1)
            %   u - control input (force on cart) (scalar)
            %
            % Outputs:
            %   dx - state derivatives (4x1)

            % Validate input dimensions
            if ~isequal(size(x), [4, 1]) && ~isequal(size(x), [1, 4])
                error('CartPendulumModel:stateFcn:InvalidState', ...
                      'State x must be a 4-element vector, got size [%d, %d]', size(x, 1), size(x, 2));
            end
            if ~isscalar(u)
                error('CartPendulumModel:stateFcn:InvalidInput', ...
                      'Input u must be a scalar, got size [%d, %d]', size(u, 1), size(u, 2));
            end

            % Ensure x is a column vector
            x = x(:);

            % extract state variables
            px    = x(1);
            theta = x(2);
            vx    = x(3);
            omega = x(4);

            % extract input variables
            F = u;

            % extract parameters
            m = obj.params.m;
            M = obj.params.M;
            l = obj.params.l;
            g = obj.params.g;

            sin_th = sin(theta);
            cos_th = cos(theta);
            cos_sin = cos_th*sin_th;
            w2 = omega^2;
            tmp = M+m*sin_th^2;

            dx1 = vx;
            dx2 = omega;
            dx3 = (-m*l*sin_th*w2+m*g*cos_sin+F)/tmp;
            dx4 = (-m*l*cos_sin*w2+F*cos_th+(M+m)*g*sin_th)/(l*tmp);

            dx = [dx1;dx2;dx3;dx4];
        end

        function y = outputFcn(obj, x, u)
            %% outputFcn - Compute output from state
            %
            % Inputs:
            %   x - state vector [px; theta; vx; omega] (4x1)
            %   u - control input (scalar)
            %
            % Outputs:
            %   y - output vector (entire state) (4x1)

            % Validate input dimensions
            if ~isequal(size(x), [4, 1]) && ~isequal(size(x), [1, 4])
                error('CartPendulumModel:outputFcn:InvalidState', ...
                      'State x must be a 4-element vector, got size [%d, %d]', size(x, 1), size(x, 2));
            end
            if ~isscalar(u)
                error('CartPendulumModel:outputFcn:InvalidInput', ...
                      'Input u must be a scalar, got size [%d, %d]', size(u, 1), size(u, 2));
            end

            % Ensure x is a column vector
            x = x(:);

            % output is entire state vector x
            y = x;
        end

        function [x_next, y_k] = simulateStep(obj, xk, uk, Ts, method, nstep)
            % Ulazni argumenti:
            % obj: instanca klase
            % xk:  trenutno stanje
            % uk:  trenutni ulaz (pretpostavlja se konstantan tijekom Ts)
            % Ts:  ukupno vrijeme koraka simulacije
            % method: 'euler' ili druge metode (trenutno samo 'euler' podržano)
            % nstep: broj internih koraka integracije

            % Argumenti koji se mogu izostaviti
            if nargin < 5
                method = 'rk4';
                nstep = 2;
            elseif nargin < 6
                switch lower(method)
                    case 'euler'
                        nstep = 10;
                    case 'rk4'
                        nstep = 2;
                    case 'ode45'
                        nstep = 1;
                    otherwise
                        warning('Unknown method %s. Supported methods: euler, rk4, ode45. Switching to rk4.', method);
                        method = 'rk4';
                        nstep = 2;
                end
            end

            switch lower(method)
                case 'euler'
                    h_step = @obj.euler_step;
                case 'rk4'
                    h_step = @obj.rk4_step;
                case 'ode45'
                    h_step = @obj.ode45_step;
                otherwise
                    warning('Unknown method %s. Supported methods: euler, rk4, ode45. Switching to rk4.', method);
                    h_step = @obj.rk4_step;
            end

            % Izračunaj izlaz y_k na *početku* intervala (vrijeme k)
            y_k = obj.outputFcn(xk, uk);

            % Izračunaj trajanje jednog internog koraka (micro-step)
            dt = Ts / nstep;

            x_current = xk; % Počinjemo od trenutnog stanja

            for i = 1:nstep
                x_current = h_step(x_current, uk, dt);
            end

            % Novo stanje nakon Ts sekundi
            x_next = x_current;
        end

        function [Ac, Bc] = stateFcnJacobian(obj, x, u)
            %% stateFcnJacobian - Compute analytical Jacobian of state function
            %
            % Computes the partial derivatives of the state dynamics with respect
            % to state and input using analytical expressions.
            %
            % Inputs:
            %   x - state vector [px; theta; vx; omega] (4x1)
            %   u - control input (force on cart) (scalar)
            %
            % Outputs:
            %   Ac - Jacobian with respect to state (4x4): ∂f/∂x
            %   Bc - Jacobian with respect to input (4x1): ∂f/∂u

            % Ensure x is column vector
            x = x(:);

            % Extract state variables
            theta = x(2);
            omega = x(4);

            % Extract parameters
            m = obj.params.m;
            M = obj.params.M;
            l = obj.params.l;
            g = obj.params.g;

            % Precompute trigonometric functions
            sin_th = sin(theta);
            cos_th = cos(theta);
            sin2_th = sin_th^2;
            cos2_th = cos_th^2;

            % Common denominator
            denom = M + m*sin2_th;
            denom2 = denom^2;

            % Derivatives for dx3/dt = (-m*l*sin(th)*omega^2 + m*g*cos(th)*sin(th) + F) / denom
            % ∂(dx3)/∂theta
            numerator_dx3 = -m*l*sin_th*omega^2 + m*g*cos_th*sin_th + u;
            d_numerator_dx3_dtheta = -m*l*cos_th*omega^2 + m*g*(cos2_th - sin2_th);
            d_denom_dtheta = 2*m*sin_th*cos_th;
            A32 = (d_numerator_dx3_dtheta*denom - numerator_dx3*d_denom_dtheta) / denom2;

            % ∂(dx3)/∂omega
            A34 = (-2*m*l*sin_th*omega) / denom;

            % Derivatives for dx4/dt = (-m*l*cos(th)*sin(th)*omega^2 + F*cos(th) + (M+m)*g*sin(th)) / (l*denom)
            % ∂(dx4)/∂theta
            numerator_dx4 = -m*l*cos_th*sin_th*omega^2 + u*cos_th + (M+m)*g*sin_th;
            d_numerator_dx4_dtheta = -m*l*(cos2_th - sin2_th)*omega^2 - u*sin_th + (M+m)*g*cos_th;
            A42 = (d_numerator_dx4_dtheta*denom - numerator_dx4*d_denom_dtheta) / (l*denom2);

            % ∂(dx4)/∂omega
            A44 = (-2*m*l*cos_th*sin_th*omega) / (l*denom);

            % Construct Ac matrix
            Ac = [0,   0,  1,   0;    % dx1 = vx
                  0,   0,  0,   1;    % dx2 = omega
                  0, A32,  0, A34;    % dx3
                  0, A42,  0, A44];   % dx4

            % Construct Bc vector (∂f/∂u)
            % dx1 and dx2 don't depend on u
            % ∂(dx3)/∂u = 1/denom
            B3 = 1 / denom;
            % ∂(dx4)/∂u = cos(theta)/(l*denom)
            B4 = cos_th / (l*denom);

            Bc = [0; 0; B3; B4];
        end

        function [Cc, Dc] = outputFcnJacobian(obj, x, u)
            %% outputFcnJacobian - Compute Jacobian of output function
            %
            % Inputs:
            %   x - state vector (4x1)
            %   u - control input (scalar)
            %
            % Outputs:
            %   Cc - Jacobian with respect to state (4x4): ∂h/∂x
            %   Dc - Jacobian with respect to input (4x1): ∂h/∂u

            Cc = eye(obj.nx);
            Dc = zeros(obj.nx, obj.nu);
        end

        function [A, B, C, D] = discreteJacobians(obj, xk, uk, Ts)
            %% discreteJacobians - Discretize continuous-time Jacobians
            %
            % Inputs:
            %   xk - state at time k (4x1)
            %   uk - input at time k (scalar)
            %   Ts - sample time (scalar)
            %
            % Outputs:
            %   A, B, C, D - discrete-time state-space matrices

            [Ac, Bc] = obj.stateFcnJacobian(xk,uk);
            [Cc, Dc] = obj.outputFcnJacobian(xk,uk);

            % Discretize using c2d
            sys_c = ss(Ac,Bc,Cc,Dc);
            sys_d = c2d(sys_c,Ts);
            A = sys_d.A; B = sys_d.B; C = sys_d.C; D = sys_d.D;
        end

        %% Getter methods for function handles
        function h = getStateFcnHandle(obj)
            %% Get handle to state function
            h = @obj.stateFcn;
        end

        function h = getOutputFcnHandle(obj)
            %% Get handle to output function
            h = @obj.outputFcn;
        end

        function h = getStateFcnJacobianHandle(obj)
            %% Get handle to state function Jacobian
            h = @obj.stateFcnJacobian;
        end

        function h = getOutputFcnJacobianHandle(obj)
            %% Get handle to output function Jacobian
            h = @obj.outputFcnJacobian;
        end

        %% Setter/getter methods for parameters
        function setParams(obj, paramName, value)
            %% Set individual parameter value
            %
            % Example: model.setParams('M', 1.5)
            if isfield(obj.params, paramName)
                obj.params.(paramName) = value;
            else
                error('CartPendulumModel:setParams:InvalidParam', ...
                      'Parameter %s does not exist', paramName);
            end
        end

        function value = getParam(obj, paramName)
            %% Get individual parameter value
            %
            % Example: M = model.getParam('M')
            if isfield(obj.params, paramName)
                value = obj.params.(paramName);
            else
                error('CartPendulumModel:getParam:InvalidParam', ...
                      'Parameter %s does not exist', paramName);
            end
        end

        function p = getAllParams(obj)
            %% Get all parameters as a structure
            p = obj.params;
        end
    end

    methods (Access = private)
        function x_next = euler_step(obj,xk,uk,h)
            %% Euler integration step
            x_next = xk + h*obj.stateFcn(xk,uk);
        end

        function x_next = ode45_step(obj,xk,uk,h)
            %% ODE45 integration step (adaptive Runge-Kutta)
            odefun = @(t,x) obj.stateFcn(x,uk);
            [~, X] = ode45(odefun, [0 h], xk);
            x_next = X(end,:)';
        end

        function x_next = rk4_step(obj,xk,uk,h)
            %% Runge-Kutta 4th order integration
            k1 = obj.stateFcn(xk, uk);
            k2 = obj.stateFcn(xk + 0.5*h*k1, uk);
            k3 = obj.stateFcn(xk + 0.5*h*k2, uk);
            k4 = obj.stateFcn(xk + h*k3, uk);
            x_next = xk + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
        end

    end

    methods (Access = private, Static)
        function p = populateParams()
            p.M = 1;    % mass of the cart [kg]
            p.m = 0.1;  % mass of the ball [kg]
            p.l = 0.8;  % length of the rod [m]
            p.g = 9.81; % gravity constant [m/s^2]
        end
    end
end
