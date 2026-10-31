package org.littletonrobotics.frc2026.util.vts.request;

import java.util.ArrayList;
import java.util.List;

// (their Lombok @Builder class, written out, with PathRequestHelpers' extension methods on it:
// segments(...) given segments adds them to what's there)
public class PathRequest {
  public final List<PathRequestSegment> segments;
  public final boolean stopAtStart, stopAtEnd;
  public final double targetDt;
  PathRequest(PathRequestBuilder b) { segments = b.segments; stopAtStart = b.stopAtStart; stopAtEnd = b.stopAtEnd; targetDt = b.targetDt; }
  public static PathRequestBuilder builder() { return new PathRequestBuilder(); }
  public PathRequestBuilder clone() { return PathRequest.builder().segments(new ArrayList<>(segments)).stopAtStart(stopAtStart).stopAtEnd(stopAtEnd); }
  public PathRequestBuilder toBuilder() { return clone(); }
  public static class PathRequestBuilder {
    List<PathRequestSegment> segments = new ArrayList<>();
    boolean stopAtStart = true, stopAtEnd = true;
    double targetDt = 0.05;
    // (PathRequestHelpers.segments with a list: appends, as their path definitions chain them;
    // clone() starts from an empty builder, so its copy is the list itself)
    public PathRequestBuilder segments(List<PathRequestSegment> s) { List<PathRequestSegment> l = new ArrayList<>(segments); l.addAll(s); segments = l; return this; }
    // (PathRequestHelpers.segments: appends)
    public PathRequestBuilder segments(PathRequestSegment... s) { List<PathRequestSegment> l = new ArrayList<>(segments); l.addAll(List.of(s)); segments = l; return this; }
    public PathRequestBuilder stopAtStart(boolean v) { stopAtStart = v; return this; }
    public PathRequestBuilder stopAtEnd(boolean v) { stopAtEnd = v; return this; }
    public PathRequestBuilder targetDt(double v) { targetDt = v; return this; }
    public PathRequest build() { return new PathRequest(this); }
  }
}
