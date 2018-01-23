function [mu, sigma] = prediction(mu, sigma, u)

dimensions = size(mu, 1);
theta_old = mu(3, 1);
Fx = [eye(3), zeros(3, dimensions - 3)];
xtmp = [u.t*cos(theta_old + u.r1); u.t*sin(theta_old + u.r1);...
    normalize_angle(u.r1 + u.r2)];
mu = mu + Fx'*xtmp;
mu(3) = normalize_angle(mu(3));

Gt = [zeros(3, 2), [-u.t*sin(theta_old + u.r1); u.t*cos(theta_old +...
    u.r1); 0]];
Gt = Fx'*Gt*Fx + eye(dimensions);

motionNoise = 0.1;
R3 = [motionNoise, 0, 0;
    0, motionNoise, 0;
    0, 0, motionNoise/10];
R = zeros(size(sigma, 1));
R(1:3, 1:3) = R3;

sigma = Gt*sigma*Gt' + R;

end