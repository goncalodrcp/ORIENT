function [estTrajs] = adjustTrajectories(trajs,t,tReal)

trajFields = fieldnames(trajs);
structFields = fieldnames(trajs.trajR); %for instance

j=1;
for i=1:length(tReal)
    
    dif = abs(t-tReal(i));
    [~,I] = min(dif);
    for n=1:numel(trajFields)
        trajectory = trajs.(trajFields{n});
        for k=1:numel(structFields)
            if(k==1)
                Rot = trajectory.(structFields{k})(:,:,I);
                estTrajs.(trajFields{n}).(structFields{k})(:,:,j)=Rot;
            else
               estTrajs.(trajFields{n}).(structFields{k})(:,j) = trajectory.(structFields{k})(:,I);
            end
        end
    end
    %trajEst_red(:,j) = traj_est(:,I);
    j=j+1;  
end


end

