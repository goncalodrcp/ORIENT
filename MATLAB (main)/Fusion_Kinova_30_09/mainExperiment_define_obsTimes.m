function obsTimes = mainExperiment_define_obsTimes(tIMU, tImages)

obsTimes = zeros(length(tIMU),1);

j=2;


for i = 2:length(tIMU)-1 %the -1 is a cheat
    if tImages(j) > tIMU(i) && tImages(j) < tIMU(i+1)
        obsTimes(i) = 1;
        j = j + 1;
        if j>length(tImages)
            break
        end
    end
end
