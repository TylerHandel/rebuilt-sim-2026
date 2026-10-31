import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Locale;
import java.util.Map;
import java.util.TreeMap;
import org.littletonrobotics.frc2026.AutoFieldConstants;
import org.littletonrobotics.frc2026.FieldConstants;
import org.littletonrobotics.frc2026.subsystems.drive.DriveTrajectories;
import org.littletonrobotics.frc2026.util.vts.request.PathRequest;
import org.littletonrobotics.frc2026.util.vts.request.PathRequestSegment;
import org.littletonrobotics.frc2026.util.vts.request.PathWaypoint;

// Their path requests (DriveTrajectories.paths) and the field poses their autos use, as JSON
public class Dump {
  static String n(double v) { return String.format(Locale.ROOT, "%.4f", v); }
  static String t(Translation2d p) { return "[" + n(p.getX()) + "," + n(p.getY()) + "]"; }
  static String p(Pose2d p) { return "[" + n(p.getX()) + "," + n(p.getY()) + "," + n(p.getRotation().getDegrees()) + "]"; }
  public static void main(String[] a) {
    StringBuilder s = new StringBuilder("{\"paths\":{");
    boolean first = true;
    for (Map.Entry<String, PathRequest> e : new TreeMap<>(DriveTrajectories.paths).entrySet()) {
      PathRequest r = e.getValue();
      s.append(first ? "" : ",").append("\n\"").append(e.getKey()).append("\":{\"stopAtStart\":").append(r.stopAtStart).append(",\"stopAtEnd\":").append(r.stopAtEnd).append(",\"segments\":[");
      first = false;
      boolean fs = true;
      for (PathRequestSegment g : r.segments) {
        s.append(fs ? "" : ",").append("{\"maxVelocity\":").append(n(g.maxVelocity)).append(",\"maxAcceleration\":").append(n(g.maxAcceleration))
            .append(",\"maxAngularVelocity\":").append(n(g.maxAngularVelocity)).append(",\"keepInLaneWidth\":").append(n(g.keepInLaneWidth));
        if (g.pointAt != null) s.append(",\"pointAt\":{\"target\":").append(t(g.pointAt.target())).append(",\"tolerance\":").append(n(g.pointAt.tolerance().getDegrees())).append(",\"flip\":").append(g.pointAt.flip()).append("}");
        if (!g.keepOutCircles.isEmpty()) s.append(",\"keepOut\":").append(g.keepOutCircles.size());
        s.append(",\"waypoints\":[");
        fs = false;
        boolean fw = true;
        for (PathWaypoint w : g.waypoints) {
          s.append(fw ? "" : ",").append("{\"xy\":").append(t(w.translation));
          if (w.rotation != null) s.append(",\"deg\":").append(n(w.rotation.getDegrees()));
          if (w.stopped) s.append(",\"stopped\":true");
          if (w.intervals >= 0) s.append(",\"intervals\":").append(w.intervals);
          s.append("}");
          fw = false;
        }
        s.append("]}");
      }
      s.append("]}");
    }
    s.append("},\n\"poses\":{");
    s.append("\"trenchLeftStart\":").append(t(AutoFieldConstants.Trench.leftStart));
    s.append(",\"trenchLeftStartOffset\":").append(t(AutoFieldConstants.Trench.leftStartOffset));
    s.append(",\"bumpLeftInner\":").append(t(AutoFieldConstants.Bump.leftInner));
    s.append(",\"bumpLeftOuter\":").append(t(AutoFieldConstants.Bump.leftOuter));
    s.append(",\"hubCenterStart\":").append(t(AutoFieldConstants.Hub.centerStart));
    s.append(",\"launchLeftBump\":").append(p(AutoFieldConstants.Launch.leftBump));
    s.append(",\"launchRightBump\":").append(p(AutoFieldConstants.Launch.rightBump));
    s.append(",\"launchLeftTrench\":").append(p(AutoFieldConstants.Launch.leftTrench));
    s.append(",\"launchLeftTower\":").append(p(AutoFieldConstants.Launch.leftTower));
    s.append(",\"climbLeft\":").append(p(AutoFieldConstants.Climb.left));
    s.append("},\n\"lines\":{");
    s.append("\"starting\":").append(n(FieldConstants.LinesVertical.starting));
    s.append(",\"neutralZoneNear\":").append(n(FieldConstants.LinesVertical.neutralZoneNear));
    s.append(",\"neutralZoneFar\":").append(n(FieldConstants.LinesVertical.neutralZoneFar));
    s.append(",\"center\":").append(n(FieldConstants.LinesVertical.center));
    s.append(",\"hubCenter\":").append(n(FieldConstants.LinesVertical.hubCenter));
    s.append(",\"leftBumpMiddle\":").append(n(FieldConstants.LinesHorizontal.leftBumpMiddle));
    s.append(",\"leftBumpStart\":").append(n(FieldConstants.LinesHorizontal.leftBumpStart));
    s.append(",\"leftBumpEnd\":").append(n(FieldConstants.LinesHorizontal.leftBumpEnd));
    s.append(",\"rightBumpStart\":").append(n(FieldConstants.LinesHorizontal.rightBumpStart));
    s.append(",\"leftTrenchMiddle\":").append(n(FieldConstants.LinesHorizontal.leftTrenchMiddle));
    s.append(",\"leftTrenchOpenEnd\":").append(n(FieldConstants.LinesHorizontal.leftTrenchOpenEnd));
    s.append(",\"rightTrenchOpenStart\":").append(n(FieldConstants.LinesHorizontal.rightTrenchOpenStart));
    s.append(",\"leftTrenchCenterX\":").append(n(FieldConstants.LeftTrench.center.getX()));
    s.append(",\"fieldWidth\":").append(n(FieldConstants.fieldWidth));
    s.append(",\"fieldLength\":").append(n(FieldConstants.fieldLength));
    s.append(",\"fuelPoolLeftCenter\":").append(t(FieldConstants.FuelPool.leftCenter));
    s.append(",\"fuelPoolRightCenter\":").append(t(FieldConstants.FuelPool.rightCenter));
    s.append(",\"hubTop\":").append(t(FieldConstants.Hub.topCenterPoint.toTranslation2d()));
    s.append("}}\n");
    System.out.print(s);
  }
}
