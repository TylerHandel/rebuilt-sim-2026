package edu.wpi.first.math.geometry;

public class Translation2d {
  public static final Translation2d kZero = new Translation2d();
  private final double x, y;
  public Translation2d() { x = 0; y = 0; }
  public Translation2d(double x, double y) { this.x = x; this.y = y; }
  public Translation2d(double d, Rotation2d a) { x = d * a.getCos(); y = d * a.getSin(); }
  public double getX() { return x; }
  public double getY() { return y; }
  public double getNorm() { return Math.hypot(x, y); }
  public Rotation2d getAngle() { return new Rotation2d(x, y); }
  public double getDistance(Translation2d o) { return Math.hypot(x - o.x, y - o.y); }
  public Translation2d plus(Translation2d o) { return new Translation2d(x + o.x, y + o.y); }
  public Translation2d minus(Translation2d o) { return new Translation2d(x - o.x, y - o.y); }
  public Translation2d times(double k) { return new Translation2d(x * k, y * k); }
  public Translation2d div(double k) { return new Translation2d(x / k, y / k); }
  public Translation2d unaryMinus() { return new Translation2d(-x, -y); }
  public Translation2d rotateBy(Rotation2d a) { return new Translation2d(x * a.getCos() - y * a.getSin(), x * a.getSin() + y * a.getCos()); }
  public Translation2d interpolate(Translation2d o, double t) { return new Translation2d(x + (o.x - x) * t, y + (o.y - y) * t); }
  public Pose2d toPose2d() { return new Pose2d(this, Rotation2d.kZero); }
}
