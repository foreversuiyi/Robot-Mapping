function plot_map(map, mapBox, robPoseMapFrame, poses, laserEndPntsMapFrame, gridSize, offset, t)
	f = figure(1);
	axis(mapBox);
	map = map';
	imshow(ones(size(map)) - log_odds_to_prob(map))
% 	traj = [poses(1:t,1)';poses(1:t,2)'];
% 	traj = world_to_map_coordinates(traj, gridSize, offset);
% 	plot(traj(1,:),traj(2,:),'g')
% 	plot(robPoseMapFrame(1),robPoseMapFrame(2),'bo','markersize',5,'linewidth',4)
% 	plot(laserEndPntsMapFrame(1,:),laserEndPntsMapFrame(2,:),'ro','markersize',2)
%     	filename = sprintf('../plots/gridmap_%03d.png', t);
% 	print(f, filename, '-dpng');
%     drawnow;
end
