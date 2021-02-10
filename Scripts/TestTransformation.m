clf;
hold on;
inputPose.X = 6;inputPose.Y = 6;inputPose.Z = 6;
positionEgo.X = 0;positionEgo.Y = 0;positionEgo.Z = 0;
orientationEgo.phi = 0;orientationEgo.theta = 0; orientationEgo.psi = pi/2;
outputPose = transformEgo(inputPose, positionEgo, orientationEgo, 0);
plot(inputPose.X, inputPose.Y,'o')
plot(outputPose.X, outputPose.Y,'x')
outputPose2 = transformEgo(outputPose, positionEgo, orientationEgo, 1);
plot(outputPose2.X, outputPose2.Y,'.')

