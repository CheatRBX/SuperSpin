package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Калибровка таблицы выстрела — главное для точности.
 * Поставьте робота на дистанцию, подберите RPM и капюшон, чтобы 5 из 5 попадали в центр ячейки,
 * перепишите строку с экрана в таблицу Shooter.java. Повторите для 6–7 дистанций.
 *
 * LB (держать) — навестись на цель, A — выстрел,
 * D-pad вверх/вниз — RPM ±50, вправо/влево — капюшон ±0.01, RT/LT — интейк.
 */
@TeleOp(name = "Shooter Tuner", group = "tune")
public class ShooterTuner extends LinearOpMode {
    @Override public void runOpMode() {
        Robot r = new Robot(hardwareMap);
        double rpm = 3000, hood = 0.35;
        boolean lu = false, ld = false, ll = false, lr = false, la = false;
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up && !lu) rpm += 50;
            if (gamepad1.dpad_down && !ld) rpm -= 50;
            if (gamepad1.dpad_right && !lr) hood = Math.min(1, hood + 0.01);
            if (gamepad1.dpad_left && !ll) hood = Math.max(0, hood - 0.01);
            if (gamepad1.a && !la) r.shooter.fire();
            lu = gamepad1.dpad_up; ld = gamepad1.dpad_down;
            ll = gamepad1.dpad_left; lr = gamepad1.dpad_right; la = gamepad1.a;

            Vision.Target t = r.vision.update(Robot.alliance);
            if (gamepad1.left_bumper && t != null) {
                double turn = Math.max(-0.3, Math.min(0.3, -(t.bearing - Robot.AIM_OFFSET_DEG) * Robot.AIM_KP));
                r.drive(0, 0, turn, false);
            } else {
                r.drive(-gamepad1.left_stick_y * 0.4, gamepad1.left_stick_x * 0.4, gamepad1.right_stick_x * 0.4, false);
            }
            r.shooter.spin(rpm);
            r.shooter.hood.setPosition(hood);
            r.shooter.update();
            r.intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

            telemetry.addData("Альянс", Robot.alliance);
            telemetry.addData("Дистанция", t == null ? "цель не видно" : String.format("%.1f\"", t.distance));
            telemetry.addData("Bearing", t == null ? "-" : String.format("%.1f°", t.bearing));
            telemetry.addData("RPM цель / L / R", "%.0f / %.0f / %.0f", rpm, r.shooter.rpmLeft(), r.shooter.rpmRight());
            telemetry.addData("На скорости", r.shooter.atSpeed());
            telemetry.addData("Капюшон", "%.2f", hood);
            telemetry.addLine(t == null ? "" : String.format("СТРОКА ТАБЛИЦЫ: %.0f, %.0f, %.2f", t.distance, rpm, hood));
            telemetry.update();
        }
        r.stopAll();
        r.vision.close();
    }
}
