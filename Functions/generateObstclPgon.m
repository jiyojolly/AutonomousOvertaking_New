function [npcVehPgons, npcVehPgonsOrg] = generateObstclPgon(npcVehLength, npcVehWidth, egoVehLength, egoVehWidth, d0VelVertex, gammaVelVertex, TfVelVertex, inflationFactor,...
                                            ActorsEgo,VehFdbk_ISO)
%GENERATEOBSTCLPGON Summary of this function goes here
%   Detailed explanation goes here
npcVehPgons.NumVeh = ActorsEgo.NumActors;
npcVehPgonsOrg.NumVeh = ActorsEgo.NumActors;
% npcVehPgonsWorld.NumVeh = npcVehPgons.NumVeh;
npcVehLengthInflated = (npcVehLength*inflationFactor) + egoVehLength;
npcVehWidthInflated = (npcVehWidth*inflationFactor) + egoVehWidth;

for i = 1:ActorsEgo.NumActors
    vEV = VehFdbk_ISO.Velocity.Xdot;
    vLV = ActorsEgo.Actors(i).Velocity(1,1);
    Tbvertex = getVelocityVertex(vLV, vEV, d0VelVertex, gammaVelVertex, TfVelVertex);
    npcLoc = [ActorsEgo.Actors(i).Position(1,1),ActorsEgo.Actors(i).Position(1,2)];
    npcVehPgonV1Org = [npcVehLength/2 npcVehLength/2 -npcVehLength/2 -npcVehLength/2;...
                     npcVehWidth/2  -npcVehWidth/2 -npcVehWidth/2  npcVehWidth/2]';
    npcVehPgonV1 = [npcVehLengthInflated/2 npcVehLengthInflated/2+Tbvertex npcVehLengthInflated/2 -npcVehLengthInflated/2 -npcVehLengthInflated/2-Tbvertex -npcVehLengthInflated/2;...
                     npcVehWidthInflated/2 0                               -npcVehWidthInflated/2 -npcVehWidthInflated/2  0                                npcVehWidthInflated/2]';
    npcVehPgonOrgVerEgo = npcVehPgonV1Org + repmat(npcLoc, 4,1);
    npcVehPgonVerEgo = npcVehPgonV1 + repmat(npcLoc, 6,1);
    npcVehPgon = rotate(polyshape(npcVehPgonVerEgo(:,1), npcVehPgonVerEgo(:,2)), ActorsEgo.Actors(i).Yaw, npcLoc);
    npcVehPgonOrg = rotate(polyshape(npcVehPgonOrgVerEgo(:,1), npcVehPgonOrgVerEgo(:,2)), ActorsEgo.Actors(i).Yaw, npcLoc);
    npcVehPgons.VehPgon(i,1).X(:,1) = npcVehPgon.Vertices(:,1);
    npcVehPgons.VehPgon(i,1).Y(:,1) = npcVehPgon.Vertices(:,2);
    npcVehPgonsOrg.VehPgon(i,1).X(:,1) = npcVehPgonOrg.Vertices(:,1);
    npcVehPgonsOrg.VehPgon(i,1).Y(:,1) = npcVehPgonOrg.Vertices(:,2);
    
end

