function npcVehPgons = generateObstclPgon(ActorsEgo, npcVehLength, npcVehWidth, egoVehLength, egoVehWidth, VehFdbk_ISO)
%GENERATEOBSTCLPGON Summary of this function goes here
%   Detailed explanation goes here
npcVehPgons.NumVeh = ActorsEgo.NumActors;
% npcVehPgonsWorld.NumVeh = npcVehPgons.NumVeh;
npcVehLengthInflated = npcVehLength + egoVehLength;
npcVehWidthInflated = npcVehWidth + egoVehWidth;

for i = 1:ActorsEgo.NumActors
    vEV = VehFdbk_ISO.Velocity.Xdot;
    vLV = ActorsEgo.Actors(i).Velocity(1,1);
    Tbvertex = getVelocityVertex(vLV,vEV);
    npcLoc = [ActorsEgo.Actors(i).Position(1,1),ActorsEgo.Actors(i).Position(1,2)];
    npcVehPgonV1 = [npcVehLengthInflated/2 npcVehLengthInflated/2 -npcVehLengthInflated/2 -npcVehLengthInflated/2-Tbvertex -npcVehLengthInflated/2;...
        npcVehWidthInflated/2 -npcVehWidthInflated/2 -npcVehWidthInflated/2 0 npcVehWidthInflated/2]';
    npcVehPgonVerEgo = npcVehPgonV1 + repmat(npcLoc, 5,1);
    npcVehPgon = rotate(polyshape(npcVehPgonVerEgo(:,1), npcVehPgonVerEgo(:,2)), ActorsEgo.Actors(i).Yaw, npcLoc);
    npcVehPgons.VehPgon(i,1).X(:,1) = npcVehPgon.Vertices(:,1);
    npcVehPgons.VehPgon(i,1).Y(:,1) = npcVehPgon.Vertices(:,2);
    
%     for j = 1:length(npcVehPgon.Vertices(:,1))
%         inputPose.X = npcVehPgon.Vertices(j,1);inputPose.Y = npcVehPgon.Vertices(j,2);inputPose.Z = 0;
%         outputPose = transformEgo(inputPose, VehFdbk_ISO.Position, VehFdbk_ISO.Orientation, 1);
%         npcVehPgonsWorld.VehPgon(i,1).X(j,1) = outputPose.X;
%         npcVehPgonsWorld.VehPgon(i,1).Y(j,1) = outputPose.Y;
%     end
    
end

