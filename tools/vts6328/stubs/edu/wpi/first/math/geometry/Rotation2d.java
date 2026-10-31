package edu.wpi.first.math.geometry;

public class Rotation2d {
  public static final Rotation2d kZero = new Rotation2d(0.0);
  public static final Rotation2d kPi = new Rotation2d(Math.PI);
  public static final Rotation2d kCW_Pi_2 = new Rotation2d(-Math.PI / 2);
  public static final Rotation2d kCCW_Pi_2 = new Rotation2d(Math.PI / 2);
  private final double r;
  public Rotation2d() { r = 0; }
  public Rotation2d(double rad) { r = Math.atan2(Math.sin(rad), Math.cos(rad)); }
  public Rotation2d(double x, double y) { r = Math.atan2(y, x); }
  public static Rotation2d fromDegrees(double d) { return new Rotation2d(Math.toRadians(d)); }
  public static Rotation2d fromRadians(double r) { return new Rotation2d(r); }
  public double getRadians() { return r; }
  public double getDegrees() { return Math.toDegrees(r); }
  public double getCos() { return Math.cos(r); }
  public double getSin() { return Math.sin(r); }
  public Rotation2d plus(Rotation2d o) { return new Rotation2d(r + o.r); }
  public Rotation2d minus(Rotation2d o) { return new Rotation2d(r - o.r); }
  public Rotation2d unaryMinus() { return new Rotation2d(-r); }
  public Rotation2d times(double k) { return new Rotation2d(r * k); }
  public Rotation2d rotateBy(Rotation2d o) { return plus(o); }
}
