function outputPose = transformEgo( inputPose, postionEgo, orientationEgo, toWorld)

rotMat = eul2rotm([orientationEgo.phi, orientationEgo.theta, orientationEgo.psi], 'XYZ');


if toWorld
    trasformMat = [[rotMat;0 0 0], [postionEgo.X postionEgo.Y postionEgo.Z 1]'];
        
else 
    trasformMat = [[rotMat';0 0 0], -[postionEgo.X postionEgo.Y postionEgo.Z 1]'];
end
outputPoseT = trasformMat*[inputPose.X inputPose.Y inputPose.Z 1]';
outputPose.X = outputPoseT(1);outputPose.Y = outputPoseT(2);outputPose.Z = outputPoseT(3); 
end
 