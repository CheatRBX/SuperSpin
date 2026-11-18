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
 * Группы AprilTag под ячейками HIVE (Competition Manual 9.9, ID 30–45).
 * Официальная библиотека SDK 12 getBioBuzzTagLibrary(): группы
 * "RED SCORING", "RED AUDIENCE", "BLUE AUDIENCE", "BLUE SCORING".
 * SDK решает положение по всей группе сразу — точнее, чем по одной метке.
 */
public class Vision {
    public enum Preference { NEAREST, SCORING, AUDIENCE }

    public static class Target {
        public final String name;
        public final double distance, bearing;  // дюймы по горизонтали; градусы, влево +
        public final int percentFound;
        public final long frameNs;               // время снимка (System.nanoTime)
        public final boolean fresh;              // новый кадр с прошлого вызова
        Target(String name, double distance, double bearing, int percentFound, long frameNs, boolean fresh) {
            this.name = name; this.distance = distance; this.bearing = bearing;
            this.percentFound = percentFound; this.frameNs = frameNs; this.fresh = fresh;
        }
        Target stale() { return new Target(name, distance, bearing, percentFound, frameNs, false); }
    }

    private final AprilTagProcessor tags;
    private final VisionPortal portal;
    private Target last = null;
    private long lastFrameNs = 0, lastSeenMs = 0;

    public Vision(HardwareMap hw) {
        tags = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        tags.setDecimation(Config.DECIMATION);

        portal = new VisionPortal.Builder()
                .setCamera(hw.get(WebcamName.class, Config.CAMERA))
                .addProcessor(tags)
                .build();
    }

    /** Вызывать раз за цикл (делает Robot.update()). null — цели давно не видно. */
    public Target update(Robot.Alliance a) {
        String color = a == Robot.Alliance.RED ? "RED" : "BLUE";
        AprilTagClusterDetection best = null;
        double bestScore = Double.MAX_VALUE;

        for (AprilTagDetection d : tags.getDetections()) {
            if (!(d instanceof AprilTagClusterDetection) || d.ftcPose == null) continue;
            AprilTagClusterDetection c = (AprilTagClusterDetection) d;
            if (c.metadata == null || !c.metadata.name.startsWith(color)) continue;
            double score = horizontal(c.ftcPose);
            if (Config.TARGET == Preference.SCORING && !c.metadata.name.contains("SCORING")) score += 1000;
            if (Config.TARGET == Preference.AUDIENCE && !c.metadata.name.contains("AUDIENCE")) score += 1000;
            if (score < bestScore) { bestScore = score; best = c; }
        }

        long nowMs = System.currentTimeMillis();
        if (best != null && best.frameAcquisitionNanoTime != lastFrameNs) {
            double dist = horizontal(best.ftcPose), bear = best.ftcPose.bearing;
            boolean continuous = last != null && last.name.equals(best.metadata.name)
                    && nowMs - lastSeenMs < Config.VISION_MAX_AGE_MS;
            if (continuous) { // сглаживаем по кадрам, а не по циклам
                dist = last.distance + Config.VISION_SMOOTH * (dist - last.distance);
                bear = last.bearing + Config.VISION_SMOOTH * (bear - last.bearing);
            }
            lastFrameNs = best.frameAcquisitionNanoTime;
            lastSeenMs = nowMs;
            last = new Target(best.metadata.name, dist, bear, best.percentClusterFound, lastFrameNs, true);
            return last;
        }
        if (last == null || nowMs - lastSeenMs > Config.VISION_MAX_AGE_MS) return null;
        return last.stale();
    }

    /** Горизонтальная дистанция: метки висят высоко, на дне ячеек HIVE. */
    static double horizontal(AprilTagPoseFtc p) {
        return p.range * Math.cos(Math.toRadians(p.elevation));
    }

    public double fps() { return portal.getFps(); }

    public void close() { portal.close(); }
}
