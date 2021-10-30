function [tReal, trajReal] = mainExperiment_correct_offset(tReal, trajReal, offset)

%correct offset

if offset ~= 1
    tReal(1:offset) = [];
    trajReal.x(:,1:offset) = []; 
    trajReal.v(:,1:offset) = [];
    trajReal.quat(1:offset) = [];
    trajReal.psi(1:offset) = [];
    trajReal.theta(1:offset) = [];
    trajReal.phi(1:offset) = [];
    trajReal.omega_b(:,1:offset) = [];
    trajReal.a_b(:,1:offset) = [];
end