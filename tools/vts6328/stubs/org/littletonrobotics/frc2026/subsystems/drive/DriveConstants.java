package org.littletonrobotics.frc2026.subsystems.drive;

import edu.wpi.first.math.util.Units;

// (their DriveConstants, Darwin's values only: what the path definitions and autos use)
public class DriveConstants {
  public static final double trackWidthX = Units.inchesToMeters(21.25);
  public static final double trackWidthY = Units.inchesToMeters(22.25);
  public static final double frameWidthX = Units.inchesToMeters(26.5);
  public static final double frameWidthY = Units.inchesToMeters(27.5);
  public static final double fullWidthX = Units.inchesToMeters(33.0);
  public static final double fullApothemX = Units.inchesToMeters(33.0) / 2.0;
  public static final double fullWidthY = Units.inchesToMeters(34.0);
  public static final double fullApothemY = Units.inchesToMeters(34.0) / 2.0;
  public static final double driveBaseRadius = Math.hypot(trackWidthX / 2.0, trackWidthY / 2.0);
  public static final double fullBaseRadius = Math.hypot(fullApothemX, fullApothemY / 2.0);
  public static final double maxLinearSpeed = 4.2;
  public static final double maxTrajectoryLinearSpeed = 4.0;
  public static final double maxAngularSpeed = 5.374;
  public static final double intakeNearX = fullApothemX;
  public static final double intakeFarX = frameWidthX / 2.0 + Units.inchesToMeters(12.0);
  public static final double intakeReferenceX = intakeNearX;
  public static final double intakeWidth = frameWidthY;
}
