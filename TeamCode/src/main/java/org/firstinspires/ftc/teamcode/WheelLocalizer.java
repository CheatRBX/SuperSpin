package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/** Одометрия по энкодерам mecanum-колёс + курс по IMU хаба. Запасной вариант без Pinpoint. */
public class WheelLocalizer implements Localizer {
    private final DcMotorEx fl, fr, bl, br;
    private final IMU imu;
    private final double ticksPerInch = Config.WHEEL_TICKS_PER_REV / (Math.PI * Config.WHEEL_DIAMETER_IN);
    private int pfl, pfr, pbl, pbr;
    private double headingOffset = 0, hv = 0;
    private Pose pose = new Pose(0, 0, 0);

    public WheelLocalizer(DcMotorEx fl, DcMotorEx fr, DcMotorEx bl, DcMotorEx br, IMU imu) {
        this.fl = fl; this.fr = fr; this.bl = bl; this.br = br; this.imu = imu;
        pfl = fl.getCurrentPosition(); pfr = fr.getCurrentPosition();
        pbl = bl.getCurrentPosition(); pbr = br.getCurrentPosition();
    }

    private double imuYaw() { return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); }

    @Override public void update() {
        int a = fl.getCurrentPosition(), b = fr.getCurrentPosition();
        int c = bl.getCurrentPosition(), d = br.getCurrentPosition();
        double dfl = (a - pfl) / ticksPerInch, dfr = (b - pfr) / ticksPerInch;
        double dbl = (c - pbl) / ticksPerInch, dbr = (d - pbr) / ticksPerInch;
        pfl = a; pfr = b; pbl = c; pbr = d;

        double fwd = (dfl + dfr + dbl + dbr) / 4;
        double left = -(dfl - dfr - dbl + dbr) / 4 * Config.STRAFE_MULT;
        double h = Pose.wrap(imuYaw() + headingOffset);
        double mid = pose.h + Pose.wrap(h - pose.h) / 2;
        pose = new Pose(
                pose.x + fwd * Math.cos(mid) - left * Math.sin(mid),
                pose.y + fwd * Math.sin(mid) + left * Math.cos(mid),
                h);
        hv = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
    }

    @Override public Pose getPose() { return pose; }

    @Override public void setPose(Pose p) {
        headingOffset = p.h - imuYaw();
        pose = p;
    }

    @Override public double headingVelocity() { return hv; }
    @Override public String name() { return "Энкодеры + IMU"; }
}
