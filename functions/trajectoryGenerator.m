function [x, v, a, t] = trajectoryGenerator(xIni, xEnd, vMax, aMax, dt)    
    % Check that the position given are 2D
    if (length(xIni) ~= 2) || (length(xEnd) ~= 2)
        error('The points given must be 2-D');
    end
    
    t = [];
    a = [];
    v = [];
    x = [];

    % tTotal = 2*vMax/aMax + (xEnd-vMax^2/aMax)/vMax;
    xI = 0;
    xE = norm(xEnd-xIni);
    
    xC = xI;
    vC = 0;

    while ((xC <= xE) || vC > 0)
        if isempty(t)
            t(1) = 0;   
        else
            t = [t, t(end) + dt];
        end

        v = [v, vC];
        x = [x, xC];

        if t(end) <= vMax/aMax
            aC = aMax;
        elseif (t(end) > vMax/aMax) && (t(end) <= vMax/aMax + (xE-vMax^2/aMax)/vMax)
            aC = 0;
        elseif t(end) > vMax/aMax + (xE-vMax^2/aMax)/vMax
            aC = -aMax;    
        end

        a = [a, aC];

        vC_aux = vC + aC*dt;
        xC_aux = xC + vC*dt + 1/2*aC*dt^2;

        vC = vC_aux;
        xC = xC_aux;
    end
    
    
    
    alpha = atand((xEnd(2) - xIni(2))/(xEnd(1) - xIni(1)));

    x = [cosd(alpha), -sind(alpha), xIni(1); sind(alpha), cosd(alpha), xIni(2); 0, 0, 1] * [x; zeros(1, length(x)); ones(1, length(x))];
    x = x(1:2,:)';
    v = [cosd(alpha), -sind(alpha); sind(alpha), cosd(alpha)] * [v; zeros(1, length(v))];
    v = v';
    a = [cosd(alpha), -sind(alpha); sind(alpha), cosd(alpha)] * [a; zeros(1, length(a))];
    a = a';
        
end