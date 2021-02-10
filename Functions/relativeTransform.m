function [outputArg1,outputArg2] = relativeTransform(translation1,orientation1,translation2,orientation2)

rotMat1 = eul2rotm(orientation1, 'XYZ');
rotMat2 = eul2rotm(orientation2, 'XYZ');

trasformMat1 = [[rotMat1;0 0 0], [translation1 1]'];
trasformMat1 = [[rotMat1;0 0 0], [translation2 1]'];
        
outputPoseT = trasformMat*[inputPose.X inputPose.Y inputPose.Z 1]';
outputPose.X = outputPoseT(1);outputPose.Y = outputPoseT(2);outputPose.Z = outputPoseT(3);
end

