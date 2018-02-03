% extract the offset of the poses and the landmarks

function [poses, landmarks] = get_poses_landmarks(g)

poses = [];
landmarks = [];
value = fields(g.idLookup);
for i = 1:length(value)
    index = str2num(value{i});
  if (index >= 100)
    offset = [0 0 0];
    poses = [poses; offset];
  else
    offset = [0 0];
    landmarks = [landmarks; offset];
  end
end

end
