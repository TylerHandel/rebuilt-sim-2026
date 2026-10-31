# prep.py CODE OUT: their FieldConstants, AutoFieldConstants and DriveTrajectories, readied to compile
# against the stand-ins (no Lombok, no AprilTag layout loader: the tags from the welded official JSON)
import json, math, os, re, sys
code, out = sys.argv[1], sys.argv[2]
J = os.path.join(code, 'src/main/java/org/littletonrobotics/frc2026')
d = json.load(open(os.path.join(code, 'src/main/deploy/apriltags/welded/2026-official.json')))
tags = []
for t in d['tags']:
    p, q = t['pose']['translation'], t['pose']['rotation']['quaternion']
    yaw = math.atan2(2 * (q['W'] * q['Z'] + q['X'] * q['Y']), 1 - 2 * (q['Y'] ** 2 + q['Z'] ** 2))
    tags.append(f"T.put({int(t['ID'])}, new double[] {{{float(p['x'])!r}, {float(p['y'])!r}, {yaw!r}}});")
src = open(os.path.join(J, 'FieldConstants.java')).read()
for imp in ['com.fasterxml.jackson.databind.ObjectMapper', 'edu.wpi.first.apriltag.AprilTagFieldLayout', 'java.io.IOException', 'java.nio.file.Path', 'lombok.Getter', 'lombok.RequiredArgsConstructor', 'org.littletonrobotics.idun.IdunPlatform']:
    src = src.replace(f'import {imp};\n', '')
i = src.index('  @RequiredArgsConstructor\n  public enum FieldType')
stub = '''  public enum FieldType { WELDED }
  // (stand-in for their layout loader: the welded official layout's tags, x, y, yaw)
  public static class TagPose {
    final double x, y, yaw;
    TagPose(double[] v) { x = v[0]; y = v[1]; yaw = v[2]; }
    public double getX() { return x; }
    public double getY() { return y; }
    public Pose2d toPose2d() { return new Pose2d(x, y, new edu.wpi.first.math.geometry.Rotation2d(yaw)); }
  }
  public static class Layout {
    static final java.util.Map<Integer, double[]> T = new java.util.HashMap<>();
    static { %s }
    public java.util.Collection<double[]> getTags() { return T.values(); }
    public double getFieldLength() { return %r; }
    public double getFieldWidth() { return %r; }
    public java.util.Optional<TagPose> getTagPose(int id) { return java.util.Optional.of(new TagPose(T.get(id))); }
  }
  public enum AprilTagLayoutType {
    OFFICIAL;
    private static final Layout L = new Layout();
    public Layout getLayout() { return L; }
  }
}
''' % ('\n    '.join(tags), float(d['field']['length']), float(d['field']['width']))
open(os.path.join(out, 'org/littletonrobotics/frc2026/FieldConstants.java'), 'w').write(src[:i] + stub)
src = open(os.path.join(J, 'AutoFieldConstants.java')).read()
open(os.path.join(out, 'org/littletonrobotics/frc2026/AutoFieldConstants.java'), 'w').write(src)
src = open(os.path.join(J, 'subsystems/drive/DriveTrajectories.java')).read()
for imp in ['lombok.experimental.ExtensionMethod', 'org.littletonrobotics.frc2026.util.geometry.GeomUtil', 'org.littletonrobotics.frc2026.util.vts.request.PathRequestHelpers']:
    src = src.replace(f'import {imp};\n', '')
src = re.sub(r'@ExtensionMethod\(\{[^}]*\}\)\n', '', src)
open(os.path.join(out, 'org/littletonrobotics/frc2026/subsystems/drive/DriveTrajectories.java'), 'w').write(src)
print('ok')
