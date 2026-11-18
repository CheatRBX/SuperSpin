package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Калибровка таблицы выстрела — главное для точности.
 * 1) Поставьте робота на дистанцию.  2) LB — навестись.  3) Подберите RPM и капюшон,
 * чтобы 5 из 5 шаров попадали в центр ячейки.  4) Перепишите «СТРОКА ТАБЛИЦЫ» в Config.
 * Повторите для 6–7 дистанций от MIN_RANGE_IN до MAX_RANGE_IN.
 *
 * Gamepad1: LB (держать) — навестись, A — выстрел, D-pad ↑/↓ — RPM ±50, →/← — капюшон ±0.01,
 *           RT/LT — интейк, стики — медленная езда. Init: X — синий, B — красный.
 */
@TeleOp(name = "Shooter Tuner", group = "tune")
public class ShooterTuner extends LinearOpMode {
    @Override public void runOpMode() {
        Robot r = new Robot(hardwareMap, new Pose(0, 0, 0));
        double rpm = 3000, hood = 0.35;

        while (opModeInInit()) {
            r.update();
            if (gamepad1.x) Robot.alliance = Robot.Alliance.BLUE;
            if (gamepad1.b) Robot.alliance = Robot.Alliance.RED;
            r.addTelemetry(telemetry);
            telemetry.update();
        }

        while (opModeIsActive()) {
            r.update();
            if (gamepad1.dpadUpWasPressed()) rpm += 50;
            if (gamepad1.dpadDownWasPressed()) rpm = Math.max(0, rpm - 50);
            if (gamepad1.dpadRightWasPressed()) hood = Math.min(1, hood + 0.01);
            if (gamepad1.dpadLeftWasPressed()) hood = Math.max(0, hood - 0.01);
            if (gamepad1.aWasPressed()) r.shooter.fire();

            if (gamepad1.left_bumper) r.aim(0, 0, false, 0, false);
            else r.drive(-gamepad1.left_stick_y * 0.4, gamepad1.left_stick_x * 0.4, gamepad1.right_stick_x * 0.4, false);

            r.shooter.spin(rpm);
            r.shooter.hood.setPosition(hood);
            r.intake(gamepad1.right_trigger - gamepad1.left_trigger);

            Vision.Target t = r.target();
            telemetry.addData("RPM / капюшон", "%.0f / %.2f", rpm, hood);
            telemetry.addData("На скорости", r.shooter.atSpeed());
            telemetry.addLine(t == null ? "СТРОКА ТАБЛИЦЫ: цель не видно"
                    : String.format("СТРОКА ТАБЛИЦЫ:  DIST %.0f   RPM %.0f   HOOD %.2f", t.distance, rpm, hood));
            r.addTelemetry(telemetry);
            telemetry.update();
        }
        r.stopAll();
        r.vision.close();
    }
}
