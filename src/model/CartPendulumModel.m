classdef PendulumModel < handle

    properties
        nx = 4;
        nu = 1;
        ny = 4;
 
        params % pendulum parameters
    end
 
    methods
        function obj = PendulumModel()
            obj.params = obj.populateParams();
        end
 
        function dx = stateFcn(obj, x, u) 
            % check dimensions x and u
 
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
           
            tmp = (M+m*(1-(cos(theta))^2)); 
 
            sin_th = sin(theta);
            cos_th = cos(theta);
            cos_sin = cos_th*sin_th;
            w2 = omega^2;
 
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
            if nargin < 6
                nstep = 1;
            end
            if nargin < 5
                method = 'euler';
            end

            % Izračunaj izlaz y_k na *početku* intervala (vrijeme k)
            y_k = obj.outputFcn(xk, uk);
            
            % Izračunaj trajanje jednog internog koraka (micro-step)
            dt = Ts / nstep; 
            
            x_current = xk; % Počinjemo od trenutnog stanja

            if strcmpi(method, 'euler')
                % Vrti petlju nstep puta
                for i = 1:nstep
                    % 1. Izračunaj derivaciju stanja (dinamiku)
                    dx = obj.stateFcn(x_current, uk);
                    
                    % 2. Primijeni Eulerov korak naprijed
                    x_current = x_current + dt * dx;
                end
            else
                % Ovdje bi se mogle dodati druge metode
                warning('Metoda "%s" nije prepoznata. Koristim "euler".', method);
                for i = 1:nstep
                    dx = obj.stateFcn(x_current, uk);
                    x_current = x_current + dt * dx;
                end
            end
            
            % Novo stanje nakon Ts sekundi
            x_next = x_current;
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