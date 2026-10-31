package edu.wpi.first.math.geometry;

public class Pose2d {
  public static final Pose2d kZero = new Pose2d();
  private final Translation2d t;
  private final Rotation2d r;
  public Pose2d() { t = Translation2d.kZero; r = Rotation2d.kZero; }
  public Pose2d(Translation2d t, Rotation2d r) { this.t = t; this.r = r; }
  public Pose2d(double x, double y, Rotation2d r) { this(new Translation2d(x, y), r); }
  public Translation2d getTranslation() { return t; }
  public Rotation2d getRotation() { return r; }
  public double getX() { return t.getX(); }
  public double getY() { return t.getY(); }
  public Pose2d plus(Transform2d o) { return transformBy(o); }
  public Pose2d transformBy(Transform2d o) { return new Pose2d(t.plus(o.getTranslation().rotateBy(r)), r.plus(o.getRotation())); }
}
