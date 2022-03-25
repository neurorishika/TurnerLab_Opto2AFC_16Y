imshow(BackgroundImage);
[width,height,~] = size(BackgroundImage);
masks = zeros(width,height,narms);
rewardmasks =  zeros(width,height,narms);

i = 1;
for arena=1:narenas
    
    disp(['Arena' num2str(arena)])
    disp('')
    disp('Select in order:'); fprintf('Between 0 & 1; Between 1 & 2; Between 2 & 0; End of 0; End of 1; End of 2\n')
    
    [x,y] = getpts;
    x = [mean(x(1:3)); x];
    y = [mean(y(1:3)); y];

    da0 = ((x(4)-x(2))^2+(y(4)-y(2))^2)^0.5;
    aa0 = atan2(y(4)-y(2),x(4)-x(2));
    a0points = [x(4) y(4); x(1) y(1); x(2) y(2); x(5)-da0*cos(aa0)/2 y(5)-da0*sin(aa0)/2; x(5)+da0*cos(aa0)/2 y(5)+da0*sin(aa0)/2];
    a0 = drawpolygon('Position',a0points, 'Color','red');
   a0rewardpoints = [(1-reward_distance)* x(4) + reward_distance  * (x(5)+da0*cos(aa0)/2) (1-reward_distance) * y(4) + reward_distance * (y(5)+da0*sin(aa0)/2);
                        x(5)+da0*cos(aa0)/2 y(5)+da0*sin(aa0)/2;
                        x(5)-da0*cos(aa0)/2 y(5)-da0*sin(aa0)/2;
                        (1-reward_distance) * x(2) + reward_distance * (x(5)-da0*cos(aa0)/2) (1-reward_distance) * y(2) + reward_distance * (y(5)-da0*sin(aa0)/2)];
    a0reward = drawpolygon('Position',a0rewardpoints, 'Color','magenta');
    
    da1 = ((x(2)-x(3))^2+(y(2)-y(3))^2)^0.5;
    aa1 = atan2(y(2)-y(3),x(2)-x(3));
    a1points = [x(2) y(2); x(1) y(1); x(3) y(3); x(6)-da1*cos(aa1)/2 y(6)-da1*sin(aa1)/2; x(6)+da1*cos(aa1)/2 y(6)+da1*sin(aa1)/2];
    a1 = drawpolygon('Position',a1points, 'Color','blue');
    a1rewardpoints = [(1-reward_distance) * x(2) + reward_distance * (x(6)+da1*cos(aa1)/2) (1-reward_distance) * y(2) + reward_distance * (y(6)+da1*sin(aa1)/2);
                        x(6)+da1*cos(aa1)/2 y(6)+da1*sin(aa1)/2;
                        x(6)-da1*cos(aa1)/2 y(6)-da1*sin(aa1)/2;
                        (1-reward_distance) * x(3) + reward_distance * (x(6)-da1*cos(aa1)/2) (1-reward_distance) * y(3) + reward_distance * (y(6)-da1*sin(aa1)/2)];
    a1reward = drawpolygon('Position',a1rewardpoints, 'Color','black');

    da2 = ((x(3)-x(4))^2+(y(3)-y(4))^2)^0.5;
    aa2 = atan2(y(3)-y(4),x(3)-x(4));
    a2points = [x(3) y(3); x(1) y(1); x(4) y(4); x(7)-da2*cos(aa2)/2 y(7)-da2*sin(aa2)/2; x(7)+da2*cos(aa2)/2 y(7)+da2*sin(aa2)/2];
    a2 = drawpolygon('Position',a2points, 'Color','green');
    a2rewardpoints = [(1-reward_distance) * x(3) + reward_distance * (x(7)+da2*cos(aa2)/2) (1-reward_distance) * y(3) + reward_distance * (y(7)+da2*sin(aa2)/2);
                        x(7)+da2*cos(aa2)/2 y(7)+da2*sin(aa2)/2;
                        x(7)-da2*cos(aa2)/2 y(7)-da2*sin(aa2)/2;
                        (1-reward_distance) * x(4) + reward_distance * (x(7)-da2*cos(aa2)/2) (1-reward_distance) * y(4) + reward_distance * (y(7)-da2*sin(aa2)/2)];
    a2reward = drawpolygon('Position',a2rewardpoints, 'Color','yellow');
    
    
    masks(:,:,i) = poly2mask(a0.Position(:,1), a0.Position(:,2),width,height);
    masks(:,:,i+1) = poly2mask(a1.Position(:,1), a1.Position(:,2),width,height);
    masks(:,:,i+2) = poly2mask(a2.Position(:,1), a2.Position(:,2),width,height);
    
    rewardmasks(:,:,i) = poly2mask(a0reward.Position(:,1), a0reward.Position(:,2),width,height);
    rewardmasks(:,:,i+1) = poly2mask(a1reward.Position(:,1), a1reward.Position(:,2),width,height);
    rewardmasks(:,:,i+2) = poly2mask(a2reward.Position(:,1), a2reward.Position(:,2),width,height);
    
    i = i + 3;
    
end

AllMasks = sum(masks,3);
MaskStack = masks;
RewardMaskStack = rewardmasks;

clear masks rewardmasks;
clear narms width height i arena x y a0 a1 a2 a0points a1points a2points a0rewardpoints a1rewardpoints a2rewardpoints a0reward a1reward a2reward
clear aa0 aa1 aa2 da0 da1 da2