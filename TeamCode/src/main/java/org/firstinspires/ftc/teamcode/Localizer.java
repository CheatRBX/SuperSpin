package org.firstinspires.ftc.teamcode;

/** Где робот. Две реализации: goBILDA Pinpoint (точнее) или энкодеры колёс + IMU. */
public interface Localizer {
    void update();
    Pose getPose();
    void setPose(Pose p);
    /** Угловая скорость, рад/с, против часовой +. */
    double headingVelocity();
    String name();
}
