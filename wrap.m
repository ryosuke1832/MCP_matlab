% Angle normalization that straightforwardly 
% returns the principal values (-π, π] of the angle

function a = wrap(a)
    a = atan2(sin(a), cos(a)); 
end


