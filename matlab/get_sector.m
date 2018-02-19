minangle = -45;
maxangle =  45;
nsectors = 5;

step = (maxangle - minangle)/nsectors;


angle = -90:1:90;
idsector = zeros(size(angle));

for i = 1:length(angle)
    if((angle(i) < minangle) || (angle(i) > maxangle))
        idsector(i) = -1;
    elseif(angle(i) == minangle)
        idsector(i) = 0;
    elseif(angle(i) == maxangle)
        idsector(i) = nsectors -1;
    else
        idsector(i) = floor((angle(i) - minangle)/step);
    end
    
end
    
    