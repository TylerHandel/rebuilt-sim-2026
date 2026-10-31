package org.littletonrobotics.frc2026.util.vts.request;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

// (their Lombok @Builder class, written out)
public class PathWaypoint {
  public final Translation2d translation;
  public final Rotation2d rotation;
  public final int intervals;
  public final boolean stopped;
  PathWaypoint(Translation2d t, Rotation2d r, int i, boolean s) { translation = t; rotation = r; intervals = i; stopped = s; }
  public static PathWaypointBuilder builder() { return new PathWaypointBuilder(); }
  public static PathWaypointBuilder from(Pose2d pose) { return builder().translation(pose.getTranslation()).rotation(pose.getRotation()); }
  public static PathWaypointBuilder from(Translation2d t) { return builder().translation(t); }
  public static class PathWaypointBuilder {
    Translation2d t; Rotation2d r = null; int i = -1; boolean s = false;
    public PathWaypointBuilder translation(Translation2d t) { this.t = t; return this; }
    public PathWaypointBuilder rotation(Rotation2d r) { this.r = r; return this; }
    public PathWaypointBuilder intervals(int i) { this.i = i; return this; }
    public PathWaypointBuilder stopped(boolean s) { this.s = s; return this; }
    public PathWaypoint build() { return new PathWaypoint(t, r, i, s); }
  }
}
