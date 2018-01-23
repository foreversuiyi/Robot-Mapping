% Turn off pagination:
more off;

clear
close all;

% how many particles
numParticles = 1000;

% initialize the particles array
particles = struct;
for i = 1:numParticles
  particles(i).weight = 1. / numParticles;
  particles(i).pose = normrnd([0 0]', [1 2]');
  particles(i).history = {};
end


% re-weight the particles according to their distance to [0 0]
sigma = diag([0.2 0.2]);
for i = 1:numParticles
  particles(i).weight = exp(-1/2 * particles(i).pose' * inv(sigma) * particles(i).pose);
end

resampledParticles = resample(particles);

% plot the particles before (red) and after resampling (blue)
bpos = [particles.pose];
apos = [resampledParticles.pose];
plot(bpos(1,:), bpos(2,:), 'r+', 'markersize', 5);
hold on
plot(apos(1,:), apos(2,:), 'b*', 'markersize', 5);
hold off
% plot(bpos(1,:), bpos(2,:), 'r+', 'markersize', 5, apos(1,:), apos(2,:), 'b*', 'markersize', 5);
