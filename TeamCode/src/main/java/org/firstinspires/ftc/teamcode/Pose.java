package org.firstinspires.ftc.teamcode;

/** Положение на поле: x вперёд, y влево (дюймы), h — курс в радианах, против часовой +. */
public final class Pose {
    public final double x, y, h;

    public Pose(double x, double y, double h) { this.x = x; this.y = y; this.h = h; }

    public static Pose deg(double x, double y, double hDeg) { return new Pose(x, y, Math.toRadians(hDeg)); }

    /** Зеркало для другого альянса: лево ↔ право. */
    public Pose mirror() { return new Pose(x, -y, -h); }

    public Pose forAlliance(Robot.Alliance a) { return a == Robot.Alliance.RED ? this : mirror(); }

    public static double wrap(double a) { return Math.atan2(Math.sin(a), Math.cos(a)); }

    @Override public String toString() {
        return String.format("(%.1f, %.1f, %.1f°)", x, y, Math.toDegrees(h));
    }
}
