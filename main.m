% Skripta za pokretanje konačne simulacije

% Kreiraj model i kontroler
model = CartPendulumModel();

% (Privremeni kontroler - samo testni signal)
controller.computeControl = @(x) 0;  % "fiktivni" kontroler

% Kreiraj simulacijsko okruženje
sim = SimulationEnvironment(model, controller, 0.01, 5, [0; 0.1; 0; 0]);

% Pokreni simulaciju
sim.run();

% Pokreni animaciju
sim.animate();
