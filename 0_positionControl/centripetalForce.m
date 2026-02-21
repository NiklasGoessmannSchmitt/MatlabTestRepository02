
scalingFactor = 0.85;
stepSize = 0.5; %(1 - scalingFactor) / 2; %0.5; 

figure
for i = 1:10 
    if scalingFactor < 0.9
        scalingFactor = scalingFactor + stepSize; 
    else
        scalingFactor = scalingFactor - stepSize; 
    end
    stepSize = stepSize / 2; 
    scalingBuffer(i) = scalingFactor; 
    
end
plot(scalingBuffer, 'r-'); grid on; hold on