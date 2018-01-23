% read a file containing ROBOTLASER1 in CARMEN logfile format
function laser=read_robotlaser(filename)

fid = fopen(filename, 'r');
laser = {};

while true
  ln = fgetl(fid);
  if (ln == -1)
    break
  end
  tokens = strsplit(ln, ' ');

  if (strcmp(tokens(1), 'ROBOTLASER1') == 0)
    continue;
  end

  num_tokens = str2double(tokens);

  currentReading = struct (...
    'start_angle', 0,...
    'angular_resolution', 0,...
    'maximum_range', 0,...
    'ranges', [],...
    'pose', zeros(3,1),...
    'laser_offset', zeros(3,1),...
    'timestamp', 0);
  tk = 3;
  currentReading.start_angle = num_tokens(tk);
  tk = tk + 2; % skip FOV
  currentReading.angular_resolution = num_tokens(tk);
  tk = tk + 1;
  currentReading.maximum_range = num_tokens(tk);
  tk = tk + 3; % skip accuracy, remission_mode

  num_readings = int32(num_tokens(tk));
  tk = tk + 1;
  currentReading.ranges = num_tokens(tk:tk+num_readings-1);
  tk = tk + num_readings;

  num_remissions = int32(num_tokens(tk)); % skip reading the remission values
  tk = tk + 1 + num_remissions;
  laser_pose = num_tokens(tk:tk+2);
  tk = tk + 3;
  currentReading.pose = num_tokens(tk:tk+2);
  tk = tk + 3;

  currentReading.laser_offset = t2v(inv(v2t(currentReading.pose)) * v2t(laser_pose));

  tk = tk + 5; % skip tv, rv, forward, side, turn
  currentReading.timestamp = num_tokens(tk);
  tk = tk + 1;
  laser{end+1} = currentReading;

end

laser = cell2mat(laser);

end
