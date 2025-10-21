clear; clc;

m = CartPendulumModel;

Nsym = 100;
x0 = [0 pi 0 0]';
Ts = 40/1000;
u = -20+40*rand(100,1);

x_euler = x0;
x_ode45 = x0;

x_history.euler = [];
x_history.ode45 = [];
for i=1:Nsym
    fprintf('Simulating step %d.\n', i)
    x_euler = m.simulateStep(x_euler, u(i), Ts, 'euler', 10);
    x_ode45 = m.simulateStep(x_ode45, u(i), Ts, 'ode45');
    x_history.euler = [x_history.euler x_euler];
    x_history.ode45 = [x_history.ode45 x_ode45];
end

plot((1:100)*Ts,x_history.euler(4,:),'r-o'); hold on; grid on;
plot((1:100)*Ts,x_history.ode45(4,:),'b-x');