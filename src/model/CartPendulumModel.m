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
            % check dimensions x and u
            % TODO!

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
            % check dimensions of x and u

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
            % Ac - jacobian of stateFcn with respect to x
            % Bc - jacobian of stateFcn with respect to u

            % x1 = x(1);
            % x2 = x(2);
            % x3 = x(3);
            % x4 = x(4);
            % 
            % cos_x2 = cos(x2);
            % 
            % A21 = (0.0100*(80*x4^2*cos_x2 - 1962*cos_x2^2 + 981))/(cos(x2)^2 - 11) - (20*cos_x2*sin(x2)*(- 0.0800*sin(x2)*x4^2 + u + 0.9810*cos(x2)*sin(x2)))/(sin(x2)^2 + 10)^2;
            % A24 = -(1.6000*x4*sin(x2))/(sin(x2)^2 + 10);
            % 
            % Ac = [0 0 1 0;0 0 0 1;
            %     0,A21, 0, A24;
            %     0, - (0.0125*(10791*cos(x2) - 160*x4^2*cos(x2)^2 - 1000*u*sin(x2) + 80*x4^2))/(cos(x2)^2 - 11) - (25*cos(x2)*sin(x2)*(- 0.0800*cos(x2)*sin(x2)*x4^2 + 10.7910*sin(x2) + u*cos(x2)))/(sin(x2)^2 + 10)^2, 0, (2*x4*sin(2*x2))/(cos(2*x2) - 21)]
            % 
            % Bc = ...
        end

        function [Cc, Dc] = outputFcnJacobian(obj, x, u)
            % CC - jacobian of outputFcn with respect to x
            % Dc - jacobian of outputFcn with respect to u
            Cc = eye(obj.nx);
            Dc = zeros(obj.nx, obj.nu);
        end

        function [A, B, C, D] = discreteJacobians(obj, xk, uk, Ts)
            [Ac, Bc] = obj.stateFcnJacobian(xk,uk);
            [Cc, Dc] = obj.outputFcnJacobian(xk,uk);
            % TODO! disretize using c2d to obtain A, B, C, D
        end
    end

    methods (Access = private)
        function x_next = euler_step(obj,xk,uk,h)
            x_next = xk + h*obj.stateFcn(xk,uk);
        end

        function x_next = ode45_step(obj,xk,uk,h)
            % TODO!
            odefun = @(t,x) obj.stateFcn(x,uk);
            [~, X] = ode45(odefun, [0 h], xk);
            x_next = X(end,:)';
        end

        function x_next = rk4_step(obj,xk,uk,h)
            % Runge-Kutta 4th order integration
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