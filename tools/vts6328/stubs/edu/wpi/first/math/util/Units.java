package edu.wpi.first.math.util;

public final class Units {
  private Units() {}
  public static double inchesToMeters(double in) { return in * 0.0254; }
  public static double degreesToRadians(double d) { return Math.toRadians(d); }
  public static double lbsToKilograms(double lb) { return lb * 0.45359237; }
}
