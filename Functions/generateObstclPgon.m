function npcVehPgons = generateObstclPgon(ActorsEgo, npcVehLength, npcVehWidth)
%GENERATEOBSTCLPGON Summary of this function goes here
%   Detailed explanation goes here
npcVehPgons.NumVeh = ActorsEgo.NumActors; 
for i = 1:ActorsEgo.NumActors
    npcLoc = [ActorsEgo.Actors(i).Position(1,1),ActorsEgo.Actors(i).Position(1,2)];
    npcVehPgonV1 = [npcVehLength/2 npcVehLength/2 -npcVehLength/2 -npcVehLength/2;...
        npcVehWidth/2 -npcVehWidth/2 -npcVehWidth/2 npcVehWidth/2]';
    npcVehPgonVerEgo = npcVehPgonV1 + repmat(npcLoc, 4,1);
    npcVehPgon = rotate(polyshape(npcVehPgonVerEgo(:,1), npcVehPgonVerEgo(:,2)), ActorsEgo.Actors(i).Yaw, npcLoc);
    npcVehPgons.VehPgon(i,1).X(:,1) = npcVehPgon.Vertices(:,1);
    npcVehPgons.VehPgon(i,1).Y(:,1) = npcVehPgon.Vertices(:,2);
end

end

