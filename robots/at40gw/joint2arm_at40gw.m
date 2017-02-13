function X = joint2arm_at40gw( Q )
%JOINT2ARM_AT40GW Returns cartesian positions of all joint locations from
%   the base to the end effector
% INPUTS:
%   Q - Mx4 vector of joint positions in degrees and mm, M is number of
%   frames
% OUTPUTS:
%   X - 6x4xM matrix of joint positions in mmsi

pe = [1; 0; 0; 0];

% TODO: Condense using arrayfun perhaps
for i = 1:size(Q,1)
    fk1 = fkine_J1_at40gw(Q(i,1),Q(i,2),Q(i,3),Q(i,4));
    fk2 = fkine_J2_at40gw(Q(i,1),Q(i,2),Q(i,3),Q(i,4));
    fk3 = fkine_J3_at40gw(Q(i,1),Q(i,2),Q(i,3),Q(i,4));
    fk4 = fkine_J4_at40gw(Q(i,1),Q(i,2),Q(i,3),Q(i,4));
    fk5 = fkine_J5_at40gw(Q(i,1),Q(i,2),Q(i,3),Q(i,4));
    fk6 = fkine_J6_at40gw(Q(i,1),Q(i,2),Q(i,3),Q(i,4));
    fk7 = fkine_tool_at40gw(Q(i,1),Q(i,2),Q(i,3),Q(i,4));
    fk8 = fkine_at40gw(Q(i,1),Q(i,2),Q(i,3),Q(i,4));
    
    %{
    tooloffsets
    a = 1214.04;
    b = 191.823;
    c = -150.149;
    
    tooloffset = [ 1 0 0 0; a 1 0 0 ; b 0 1 0; c 0 0 1]; 
    %}
     
    
    X0 = [pe fk1*pe fk2*pe fk3*pe fk4*pe fk5*pe fk6*pe fk7*pe fk8*pe];

    X(:,:,i) = X0(2:4,:)';
    
end

end

