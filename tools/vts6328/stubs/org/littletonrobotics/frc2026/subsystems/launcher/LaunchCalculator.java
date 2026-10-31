package org.littletonrobotics.frc2026.subsystems.launcher;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.frc2026.FieldConstants;

// (their getStationaryAimedPose, forceBlue, not passing: the launcher, on the robot's centerline
// facing back, at the HUB's top center; getDriveAngleWithLauncherOffset with robotToLauncher's y 0
// and its yaw pi)
public class LaunchCalculator {
  public static Pose2d getStationaryAimedPose(Translation2d robotTranslation, boolean forceBlue) {
    Translation2d target = FieldConstants.Hub.topCenterPoint.toTranslation2d();
    Rotation2d fieldToHubAngle = target.minus(robotTranslation).getAngle();
    return new Pose2d(robotTranslation, fieldToHubAngle.plus(Rotation2d.kPi));
  }
}
