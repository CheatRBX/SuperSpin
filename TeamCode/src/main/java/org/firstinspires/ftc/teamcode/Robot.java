package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/** Общее железо робота. Имена должны совпадать с конфигурацией на Driver Hub. */
public class Robot {
    public DcMotorEx fl, fr, bl, br, lift, intake;
    public Servo claw;
    public IMU imu;

    public static final double CLAW_OPEN = 0.35, CLAW_CLOSED = 0.65;
    public static final int LIFT_MIN = 0, LIFT_MAX = 3000; // тики энкодера, подстройте
    public static final double TICKS_PER_INCH = 537.7 / (Math.PI * 4.09); // goBILDA 312rpm, колесо 104мм

    public Robot(HardwareMap hw) {
        fl = hw.get(DcMotorEx.class, "fl");
        fr = hw.get(DcMotorEx.class, "fr");
        bl = hw.get(DcMotorEx.class, "bl");
        br = hw.get(DcMotorEx.class, "br");
        lift = hw.get(DcMotorEx.class, "lift");
        intake = hw.get(DcMotorEx.class, "intake");
        claw = hw.get(Servo.class, "claw");
        imu = hw.get(IMU.class, "imu");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br, lift}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();
    }

    public double heading() { return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); }

    /** Mecanum: y вперёд, x вбок, r поворот. fieldCentric — относительно поля. */
    public void drive(double y, double x, double r, boolean fieldCentric) {
        if (fieldCentric) {
            double h = -heading();
            double nx = x * Math.cos(h) - y * Math.sin(h);
            double ny = x * Math.sin(h) + y * Math.cos(h);
            x = nx * 1.1; y = ny;
        }
        double d = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);
        fl.setPower((y + x + r) / d);
        bl.setPower((y - x + r) / d);
        fr.setPower((y - x - r) / d);
        br.setPower((y + x - r) / d);
    }

    public void stop() { drive(0, 0, 0, false); }

    public void liftTo(int ticks, double power) {
        lift.setTargetPosition(Math.max(LIFT_MIN, Math.min(LIFT_MAX, ticks)));
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(power);
    }
}
