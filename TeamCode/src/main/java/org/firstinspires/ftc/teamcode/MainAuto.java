package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "BioBuzz Auto")
public class MainAuto extends LinearOpMode {
    Robot r;

    @Override public void runOpMode() {
        r = new Robot(hardwareMap);
        r.claw.setPosition(Robot.CLAW_CLOSED);
        waitForStart();

        // Пример маршрута — замените под задание сезона
        move(24, 0, 0.6);          // вперёд 24"
        r.liftTo(Robot.LIFT_MAX, 1);
        sleep(1200);
        r.claw.setPosition(Robot.CLAW_OPEN);
        sleep(400);
        r.liftTo(Robot.LIFT_MIN, 1);
        turnTo(90, 0.5);
        move(0, 30, 0.6);          // вбок 30" (парковка)
    }

    /** Движение по энкодерам с удержанием курса. fwd/strafe в дюймах. */
    void move(double fwd, double strafe, double power) {
        DcMotorEx[] m = {r.fl, r.fr, r.bl, r.br};
        int[] t = {
            (int) ((fwd + strafe) * Robot.TICKS_PER_INCH), (int) ((fwd - strafe) * Robot.TICKS_PER_INCH),
            (int) ((fwd - strafe) * Robot.TICKS_PER_INCH), (int) ((fwd + strafe) * Robot.TICKS_PER_INCH)};
        for (int i = 0; i < 4; i++) {
            m[i].setTargetPosition(m[i].getCurrentPosition() + t[i]);
            m[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m[i].setPower(power);
        }
        while (opModeIsActive() && (r.fl.isBusy() || r.fr.isBusy() || r.bl.isBusy() || r.br.isBusy())) idle();
        for (DcMotorEx x : m) { x.setPower(0); x.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); }
    }

    /** P-регулятор поворота по IMU. */
    void turnTo(double deg, double maxPower) {
        double target = Math.toRadians(deg);
        while (opModeIsActive()) {
            double err = Math.atan2(Math.sin(target - r.heading()), Math.cos(target - r.heading()));
            if (Math.abs(err) < Math.toRadians(1.5)) break;
            double p = Math.max(-maxPower, Math.min(maxPower, err * 1.2));
            if (Math.abs(p) < 0.08) p = Math.copySign(0.08, p);
            r.drive(0, 0, -p, false);
        }
        r.stop();
    }
}
