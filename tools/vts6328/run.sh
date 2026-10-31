#!/bin/sh
# 6328's path requests (their DriveTrajectories.paths) to tools/vts6328/paths6328.json, for
# tools/autos6328.py. Their DriveTrajectories, FieldConstants and AutoFieldConstants compiled as they
# are (prep.py: Lombok's annotations off, their AprilTag layout loader replaced by its welded official
# layout's tags) against stubs/: WPILib's geometry, their DriveConstants (Darwin's values),
# LaunchCalculator.getStationaryAimedPose, VerticalFlipUtil and the path request builders (Lombok's
# @Builder and PathRequestHelpers' extension methods, written out). Needs a JDK.
#   tools/vts6328/run.sh CODE   (CODE: a checkout of Mechanical-Advantage/RobotCode2026Public)
set -e
here=$(cd "$(dirname "$0")" && pwd)
work=$(mktemp -d)
cp -r "$here/stubs/." "$work/"
mkdir -p "$work/org/littletonrobotics/frc2026/subsystems/drive"
python3 -I "$here/prep.py" "$1" "$work"
cp "$here/Dump.java" "$work/"
find "$work" -name '*.java' > "$work/files.txt"
javac -d "$work/out" @"$work/files.txt"
java -cp "$work/out" Dump > "$here/paths6328.json"
rm -rf "$work"
echo "wrote $here/paths6328.json"
