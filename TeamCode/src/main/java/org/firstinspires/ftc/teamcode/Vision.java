package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * Камера видит группы AprilTag под ячейками HIVE (Competition Manual 9.9, ID 30–45).
 * Используется официальная библиотека SDK 12 — getBioBuzzTagLibrary(): там 4 группы
 * "RED SCORING", "RED AUDIENCE", "BLUE AUDIENCE", "BLUE SCORING".
 * SDK решает положение по всей группе сразу (до 16 углов) — это точнее одной метки.
 */
public class Vision {
    public enum Preference { NEAREST, SCORING, AUDIENCE }

    /** Какую ячейку своего альянса выбирать. Проверьте на поле, какая видна с точки выстрела. */
    public static Preference target = Preference.NEAREST;
    public static double SMOOTH = 0.5;     // 0..1: больше — быстрее реакция, меньше — меньше дрожь
    public static long MAX_AGE_MS = 250;   // сколько верить последнему кадру

    public static class Target {
        public final String name;
        public final double distance, bearing;
        public final int percentFound;
        Target(String name, double distance, double bearing, int percentFound) {
            this.name = name; this.distance = distance;
            this.bearing = bearing; this.percentFound = percentFound;
        }
    }

    private final AprilTagProcessor tags;
    private final VisionPortal portal;
    private Target last = null;
    private long lastSeenMs = 0;

    public Vision(HardwareMap hw) {
        tags = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        tags.setDecimation(2);

        portal = new VisionPortal.Builder()
                .setCamera(hw.get(WebcamName.class, "Webcam 1"))
                .addProcessor(tags)
                .build();
    }

    /** Лучшая видимая группа своего альянса или null, если давно не видно. */
    public Target update(Robot.Alliance a) {
        String color = a == Robot.Alliance.RED ? "RED" : "BLUE";
        AprilTagClusterDetection best = null;
        double bestScore = Double.MAX_VALUE;

        for (AprilTagDetection d : tags.getDetections()) {
            if (!(d instanceof AprilTagClusterDetection) || d.ftcPose == null) continue;
            AprilTagClusterDetection c = (AprilTagClusterDetection) d;
            if (c.metadata == null || !c.metadata.name.startsWith(color)) continue;

            double score = horizontal(c.ftcPose);
            if (target == Preference.SCORING && !c.metadata.name.contains("SCORING")) score += 1000;
            if (target == Preference.AUDIENCE && !c.metadata.name.contains("AUDIENCE")) score += 1000;
            if (score < bestScore) { bestScore = score; best = c; }
        }

        long now = System.currentTimeMillis();
        if (best != null) {
            double dist = horizontal(best.ftcPose), bear = best.ftcPose.bearing;
            if (last != null && last.name.equals(best.metadata.name) && now - lastSeenMs < MAX_AGE_MS) {
                dist = last.distance + SMOOTH * (dist - last.distance);
                bear = last.bearing + SMOOTH * (bear - last.bearing);
            }
            last = new Target(best.metadata.name, dist, bear, best.percentClusterFound);
            lastSeenMs = now;
        }
        return (last != null && now - lastSeenMs < MAX_AGE_MS) ? last : null;
    }

    /** Горизонтальная дистанция: метки висят высоко (дно ячеек HIVE). */
    static double horizontal(AprilTagPoseFtc p) {
        return p.range * Math.cos(Math.toRadians(p.elevation));
    }

    public void close() { portal.close(); }
}
