package edu.wpi.first.math.geometry;

public class Translation3d {
  private final double x, y, z;
  public Translation3d(double x, double y, double z) { this.x = x; this.y = y; this.z = z; }
  public double getX() { return x; }
  public double getY() { return y; }
  public double getZ() { return z; }
  public Translation2d toTranslation2d() { return new Translation2d(x, y); }
}
