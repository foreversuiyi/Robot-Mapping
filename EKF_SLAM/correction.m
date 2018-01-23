function [mu, sigma, observed_landmarks] = correction(mu, sigma, z, observed_landmarks)
m = size(z, 2);
dimension = size(mu, 1);

Z = zeros(m*2, 1);
expected_Z = zeros(m*2, 1);
H = [];

for i = 1:m
    landmark_id = z(i).id;
    if(observed_landmarks(landmark_id) == false)
        mu(2*landmark_id + 2 : 2*landmark_id + 3, 1) = ...
            mu(1:2, 1) + [z(i).range*cos(z(i).bearing + mu(3, 1));...
            z(i).range*sin(z(i).bearing + mu(3, 1))];
        observed_landmarks(landmark_id) = true;
    end
    
    Z(2*i - 1 : 2*i, 1) = [z(i).range; z(i).bearing];
    del = mu(2*landmark_id + 2 : 2*landmark_id + 3, 1) - mu(1:2, 1);
    q = del'*del;
    expected_Z(2*i - 1 : 2*i, 1) = [sqrt(q); normalize_angle(atan2(del(2,1), del(1,1)) - mu(3,1))];
    Fx = zeros(5, dimension);
    Fx(1:3, 1:3) = eye(3);
    Fx(4, 2*landmark_id + 2) = 1;
    Fx(5, 2*landmark_id + 3) = 1;
    Hi = 1/q*[-sqrt(q)*del(1,1), -sqrt(q)*del(2,1), 0, sqrt(q)*del(1,1), sqrt(q)*del(2,1);
        del(2,1), -del(1,1), -q, -del(2,1), del(1,1)];
    Hi = Hi*Fx;
    H = [H; Hi];
%     Q = 0.01 * eye(2);
%     K = sigma*Hi'/(Hi*sigma*Hi'+Q);
%     dz = normalize_all_bearings(Z - expected_Z);
%     mu = mu + K*dz;
%     sigma = (eye(dimension) - K*Hi)*sigma;
end
Q = 0.01 * eye(2*m);
K = sigma*H'/(H*sigma*H'+Q);
dz = normalize_all_bearings(Z - expected_Z);
mu = mu + K*dz;
sigma = (eye(dimension) - K*H)*sigma;
end