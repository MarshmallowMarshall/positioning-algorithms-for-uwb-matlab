function yk = citrackMeasurementFcn(xk)

% Inputs:
%    xk - x[k], states at time k
%
% Outputs:
%    yk - y[k], measurements at time k


% Known anchors Positions in 2D at TWB
A0_2d = [4.22, -0.09];          
A1_2d = [3.72, 1.71]; 
A2_2d = [0, 1.5];
A3_2d = [0, 0];

% Known anchors positions in Sporthall
% A0_2d = [0, 0];          
% A1_2d = [20, 0]; 
% A2_2d = [20, 20];
% A3_2d = [0, 20];

Anc_2D = [A0_2d; A1_2d; A2_2d; A3_2d];
[nAnc, nDim] = size(Anc_2D);

yk = zeros(nAnc, 1);  % There are 4 measurements from 4 anchors and a tag
% yk = zeros(size(xk)); % TODO: this seems incorrect for CA case

for jj = 1 : nAnc
    % This is the delta range, i.e. the measured minus the best
    % known range values
    yk(jj) = sqrt((Anc_2D(jj, 1) - xk(1)).^2 + (Anc_2D(jj, 2) - xk(2)).^2);    
end


%{
% This is how it would be done in Linear case 

% The measurement transition matrix 
H = [ 1  1  0  0;
      1  1  0  0;
      1  1  0  0;
      1  1  0  0];
      
yk = H * xk;
%}

end