I = imread('./data/EWN_cropped.png'); 
% 1462 x 884
% länge = 30
% --> breite = 30/1462 * 884 = 18.1395

%% 
figure
h = image([0 30],[0 26], I);
uistack(h,'bottom')
daspect([1 1 1])