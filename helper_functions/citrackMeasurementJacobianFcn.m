function dhdx = citrackMeasurementJacobianFcn(xk)  

% dhdx = vdpMeasurementJacobianFcn(x)
%
% Inputs:
%    x - States x[k]
%
% Outputs:
%    dhdx - dh/dx, the Jacobian of citrackMeasurementFcn evaluated at x[k]
%


%#codegen

% The tag %#codegen must be included if you wish to generate code with 
% MATLAB Coder.
% dhdx = [1 0];

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

state_vec_len = length(xk);
dhdx = zeros(nAnc, state_vec_len);

ri_0 = zeros(nAnc, 1);

for jj = 1 : nAnc
    % This is the Jacobian Matrix for measurement 
%     dhdx(jj) = sqrt((Anc_2D(jj, 1) - xk(1)).^2 + (Anc_2D(jj, 2) - xk(2)).^2); 
    ri_0(jj) = sqrt((Anc_2D(jj, 1) - xk(1)).^2 + (Anc_2D(jj, 2) - xk(2)).^2);
    
    if (state_vec_len == 4) % 2D Constant accelaration model
        dhdx(jj, 1) = (xk(1) - Anc_2D(jj, 1))./ ri_0(jj);
        dhdx(jj, 2) = (xk(2) - Anc_2D(jj, 2))./ ri_0(jj);
        dhdx(jj, 3) = 0;           % no velocity measurement data
        dhdx(jj, 4) = 0;
        
    elseif (state_vec_len == 6) 
        dhdx(jj, 1) = (xk(1) - Anc_2D(jj, 1))./ ri_0(jj);
        dhdx(jj, 2) = (xk(2) - Anc_2D(jj, 2))./ ri_0(jj);
        dhdx(jj, 3) = 0;  % no velocity measurement data
        dhdx(jj, 4) = 0;
        dhdx(jj, 5) = 0;  % No acceleration data available in the measurement 
        dhdx(jj, 6) = 0; 
    else
        msg = "Error: 3D state vector is not implemented yet!\n";
        error(msg);    
    end
end

end