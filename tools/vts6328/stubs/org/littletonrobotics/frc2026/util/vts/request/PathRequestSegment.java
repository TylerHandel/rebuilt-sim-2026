package org.littletonrobotics.frc2026.util.vts.request;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;

// (their Lombok @Builder class, written out, with PathRequestHelpers' extension methods on it)
public class PathRequestSegment {
  public final List<PathWaypoint> waypoints;
  public final double maxVelocity, maxAcceleration, maxAngularVelocity, keepInLaneWidth;
  public final PointTarget pointAt;
  public final List<KeepOutCircle> keepOutCircles;
  PathRequestSegment(PathRequestSegmentBuilder b) {
    waypoints = b.waypoints; maxVelocity = b.maxVelocity; maxAcceleration = b.maxAcceleration; maxAngularVelocity = b.maxAngularVelocity;
    keepInLaneWidth = b.keepInLaneWidth; pointAt = b.pointAt; keepOutCircles = b.keepOutCircles;
  }
  public static PathRequestSegmentBuilder builder() { return new PathRequestSegmentBuilder(); }
  public PathRequestSegmentBuilder toBuilder() {
    PathRequestSegmentBuilder b = new PathRequestSegmentBuilder();
    b.waypoints = waypoints; b.maxVelocity = maxVelocity; b.maxAcceleration = maxAcceleration; b.maxAngularVelocity = maxAngularVelocity;
    b.keepInLaneWidth = keepInLaneWidth; b.pointAt = pointAt; b.keepOutCircles = keepOutCircles;
    return b;
  }
  public static record PointTarget(Translation2d target, Rotation2d tolerance, Boolean flip) {}
  public static record KeepOutCircle(Translation2d center, double radius) {}
  public static class PathRequestSegmentBuilder {
    List<PathWaypoint> waypoints = new ArrayList<>();
    double maxVelocity = -1, maxAcceleration = -1, maxAngularVelocity = -1, keepInLaneWidth = -1;
    PointTarget pointAt = null;
    List<KeepOutCircle> keepOutCircles = new ArrayList<>();
    public PathRequestSegmentBuilder waypoints(List<PathWaypoint> w) { waypoints = w; return this; }
    public PathRequestSegmentBuilder waypoints(PathWaypoint... w) { waypoints = new ArrayList<>(List.of(w)); return this; }
    public PathRequestSegmentBuilder maxVelocity(double v) { maxVelocity = v; return this; }
    public PathRequestSegmentBuilder maxAcceleration(double v) { maxAcceleration = v; return this; }
    public PathRequestSegmentBuilder maxAngularVelocity(double v) { maxAngularVelocity = v; return this; }
    public PathRequestSegmentBuilder keepInLaneWidth(double v) { keepInLaneWidth = v; return this; }
    public PathRequestSegmentBuilder straightLine() { keepInLaneWidth = 0.01; return this; }
    public PathRequestSegmentBuilder pointAt(PointTarget p) { pointAt = p; return this; }
    public PathRequestSegmentBuilder keepOutCircles(List<KeepOutCircle> c) { keepOutCircles = c; return this; }
    public PathRequestSegment build() { return new PathRequestSegment(this); }
  }
}
