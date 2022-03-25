for i=1:12
    for j=1:12
        overlap = MaskStack(:,:,i)&MaskStack(:,:,j);
        if any(any(overlap)) && i~=j
            disp(['Overlap found between Mask ' num2str(i) ' and Mask ' num2str(j) ' in the Arm Masks. Overlap will be removed from Mask ' num2str(i)])
            MaskStack(:,:,i) = MaskStack(:,:,i) - overlap;
        end
    end
end

for i=1:12
    for j=1:12
        overlap = RewardMaskStack(:,:,i)&RewardMaskStack(:,:,j);
        if any(any(overlap)) && i~=j
            disp(['Overlap found between Mask ' num2str(i) ' and Mask ' num2str(j) ' in the Reward Masks. Overlap will be removed from Mask ' num2str(i)])
            RewardMaskStack(:,:,i) = RewardMaskStack(:,:,i) - overlap;
        end
    end
end

clear overlap i j