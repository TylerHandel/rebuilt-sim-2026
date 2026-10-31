package edu.wpi.first.math.geometry;

public class Transform2d {
  private final Translation2d t;
  private final Rotation2d r;
  public Transform2d(Translation2d t, Rotation2d r) { this.t = t; this.r = r; }
  public Transform2d(double x, double y, Rotation2d r) { this(new Translation2d(x, y), r); }
  public Translation2d getTranslation() { return t; }
  public Rotation2d getRotation() { return r; }
}
