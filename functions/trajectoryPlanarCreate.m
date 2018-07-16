function [ref, ref_d, ref_dd, t] = trajectoryPlanarCreate(trajType)
    
    if strcmp(trajType, 'LinearX')
        
        % Defining vertices
        xIni = 1.05; xEnd = 1.05;
        yIni = 0.8; yEnd = -0.2;
        
        % Velocity and acceleration parameters
        dt = 0.1;
        vMax = 0.05;
        aMax = 0.005;
        
        % Trajectroy - Edge 1
        [refAux, refAux_d, refAux_dd, t] = trajectoryGenerator(0, abs(yEnd-yIni), vMax, aMax, dt);
        ref = [cosd(-90), -sind(-90), xIni; sind(-90), cosd(-90), yIni; 0, 0, 1] * [refAux; zeros(1, length(refAux)); ones(1, length(refAux))];
        ref = ref(1:2,:)';
        ref_d = [cosd(-90), -sind(-90); sind(-90), cosd(-90)] * [refAux_d; zeros(1, length(refAux_d))];
        ref_d = ref_d';
        ref_dd = [cosd(-90), -sind(-90); sind(-90), cosd(-90)] * [refAux_dd; zeros(1, length(refAux_dd))];
        ref_dd = ref_dd';
        
    elseif strcmp(trajType, 'Tria')
        
        xIni = 0; xMid = 1; xEnd = 0.5;
        yIni = 1*sind(60) - 0.16; yMid = 1*sind(60) - 0.16; yEnd = -0.16;
        d2r = pi/180;
        thIni = 90*d2r; thMid = 90*d2r; thEnd = 90*d2r;

        dt = 0.1;
        vMax = 0.05;
        aMax = 0.005;
        
        % Trajectroy - Edge 1
        [ref1, ref1_d, ref1_dd, t1] = trajectoryGenerator(xIni, xMid, vMax, aMax, dt);
        ref1 = [ref1', yIni*ones(length(ref1), 1)];
        ref1_d = [ref1_d', zeros(length(ref1), 1)];
        ref1_dd = [ref1_dd', zeros(length(ref1), 1)];

        % Trajectory - Edge 2
        [refAux, refAux_d, refAux_dd, tAux] = trajectoryGenerator(xIni, xMid, vMax, aMax, dt);
        ref2 = [cosd(-120), -sind(-120), ref1(end, 1); sind(-120), cosd(-120), ref1(end, 2); 0, 0, 1] * [refAux; zeros(1, length(ref1)); ones(1, length(ref1))];
        ref2 = ref2(1:2,:)';
        ref2_d = [cosd(-120), -sind(-120); sind(-120), cosd(-120)] * [refAux_d; zeros(1, length(refAux_d))];
        ref2_d = ref2_d';
        ref2_dd = [cosd(-120), -sind(-120); sind(-120), cosd(-120)] * [refAux_dd; zeros(1, length(refAux_dd))];
        ref2_dd = ref2_dd';
        % Deleting the first row as it is the last of the Edge 1
        ref2(1, :) = []; ref2_d(1, :) = []; ref2_dd(1, :) = []; tAux(1) = [];
        
        % Trajectroy - Total
        ref = [ref1; ref2]; 
        ref_d = [ref1_d; ref2_d]; 
        ref_dd = [ref1_dd; ref2_dd];

        t = [t1, t1(end) + tAux];
    end
    

end
