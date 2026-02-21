function [Layer_all] = possibleLayers(Layout)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
%layerArray=string;
%layerArray=[];
layerArray =[convertCharsToStrings(Layout.entities(1).layer)];
%layerArray =[layerArray, convertCharsToStrings (Layout.entities(1).layer);
k=1;


for i=1:Layout.ne
    if any(strcmp(Layout.entities(i).layer,layerArray))
        % current layer is alreay part of layer array --> do nothing

    else
        % add this layer to layer array 
        layerArray = [layerArray, convertCharsToStrings(Layout.entities(i).layer)];
    end
end



% for i=1:ne
%     if ~any(strcmp({Layout.entities.layer(i)},text))
%         % Do Something
% 
%     else
%         % Do Something else
%         Layer[k]={Layout.entities.layer(i)};
%         k=k+1;
%         text= {Layout.entities.layer(i)};
%         
%     end
% end
Layer_all=[convertCharsToStrings( layerArray)];
end