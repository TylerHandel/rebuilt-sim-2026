package org.littletonrobotics.frc2026.util.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.frc2026.FieldConstants;

public class VerticalFlipUtil {
  public static double applyY(double y) { return FieldConstants.fieldWidth - y; }
  public static Translation2d apply(Translation2d t) { return new Translation2d(t.getX(), applyY(t.getY())); }
  public static Rotation2d apply(Rotation2d r) { return r.times(-1.0); }
  public static Pose2d apply(Pose2d p) { return new Pose2d(apply(p.getTranslation()), apply(p.getRotation())); }
}
