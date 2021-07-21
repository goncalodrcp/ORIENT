function [state,cov] = stateStruct(Rot,v,o,b_g,b_a,pLand,P)
    %Define state variables in a structure
    state.Rot = Rot;
    state.v = v;
    state.o = o;
    state.b_g = b_g;
    state.b_a = b_a;
    state.pLand = pLand;
    %State uncertainty associated with covariance matrix P
    cov = P;
end

