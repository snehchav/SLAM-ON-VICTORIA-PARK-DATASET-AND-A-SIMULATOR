% highlights the detectedmarker id

function plotfield( detectedMarkers)
% PLOTFIELD

global FIELDINFO;

WAS_HOLD = ishold;

if ~WAS_HOLD
    hold on
end

margin = 200;

axis equal;
axis([-margin, FIELDINFO.COMPLETE_SIZE_X+margin, -margin, FIELDINFO.COMPLETE_SIZE_Y+margin]);


nMarkers = length(FIELDINFO.MARKER_X_POS);
for i = 1:nMarkers
    if any(i == detectedMarkers)
        plotcircle([FIELDINFO.MARKER_X_POS(i), FIELDINFO.MARKER_Y_POS(i)], ...
            15, 200, 'black',1, [0.8 0.8 0.8]);
    else
        plotcircle([FIELDINFO.MARKER_X_POS(i), FIELDINFO.MARKER_Y_POS(i)], ...
            15, 200, 'black',1, 'white');
    end
    plot(FIELDINFO.MARKER_X_POS(i), FIELDINFO.MARKER_Y_POS(i), 'ko', 'MarkerSize',2);
    text(FIELDINFO.MARKER_X_POS(i)-2, FIELDINFO.MARKER_Y_POS(i)+2, num2str(i));
end

if ~WAS_HOLD
    hold off
end

