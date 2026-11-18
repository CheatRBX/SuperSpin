package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Gamepad1 (водитель):
 *   левый стик — езда, правый стик X — поворот, RT — медленно,
 *   RB (держать) — автонаведение + автовыстрел (стой на месте),
 *   LB (держать) — то же + робот сам подъезжает на рабочую дистанцию,
 *   BACK — field/robot-centric, START — сброс IMU.
 * Gamepad2 (оператор):
 *   RT — интейк внутрь, LT — наружу, Y — предраскрутка вкл/выкл, B — маховик стоп,
 *   X — ручной выстрел, D-pad вверх/вниз — поправка оборотов ±50,
 *   A (держать) — запасной выстрел без камеры по дистанции 54".
 * Init: X — синий альянс, B — красный.
 */
@TeleOp(name = "BioBuzz TeleOp")
public class MainTeleOp extends LinearOpMode {
    @Override public void runOpMode() {
        Robot r = new Robot(hardwareMap);
        boolean field = true, prespin = true;
        boolean lastBack = false, lastY = false, lastUp = false, lastDown = false;

        while (opModeInInit()) {
            if (gamepad1.x || gamepad2.x) Robot.alliance = Robot.Alliance.BLUE;
            if (gamepad1.b || gamepad2.b) Robot.alliance = Robot.Alliance.RED;
            Vision.Target t = r.vision.update(Robot.alliance);
            telemetry.addData("Альянс (X синий / B красный)", Robot.alliance);
            telemetry.addData("HIVE видно", t == null ? "нет" : String.format("%.0f\"", t.distance));
            telemetry.update();
        }

        while (opModeIsActive()) {
            // --- переключатели ---
            if (gamepad1.back && !lastBack) field = !field;
            lastBack = gamepad1.back;
            if (gamepad1.start) r.imu.resetYaw();
            if (gamepad2.y && !lastY) prespin = !prespin;
            lastY = gamepad2.y;
            if (gamepad2.dpad_up && !lastUp) Shooter.rpmTrim += 50;
            if (gamepad2.dpad_down && !lastDown) Shooter.rpmTrim -= 50;
            lastUp = gamepad2.dpad_up;
            lastDown = gamepad2.dpad_down;

            double slow = gamepad1.right_trigger > 0.3 ? 0.35 : 1.0;
            double y = -gamepad1.left_stick_y * slow, x = gamepad1.left_stick_x * slow;
            boolean ready = false;
            boolean aiming = gamepad1.right_bumper || gamepad1.left_bumper;

            if (aiming) {
                ready = r.aimStep(y, x, gamepad1.left_bumper, 0);
                if (ready) r.shooter.fire();
            } else {
                r.drive(y, x, gamepad1.right_stick_x * slow, field);
                if (gamepad2.a) r.shooter.setDistance(54);
                else if (gamepad2.b) r.shooter.stop();
                else if (prespin) r.shooter.idle();
                else r.shooter.stop();
            }

            if (gamepad2.x) r.shooter.fire(); // ручной выстрел
            r.intake.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
            r.shooter.update();

            Vision.Target t = r.vision.update(Robot.alliance);
            telemetry.addData("Альянс", Robot.alliance);
            telemetry.addData("Режим", aiming ? (ready ? "ОГОНЬ" : "наведение") : (field ? "field-centric" : "robot-centric"));
            telemetry.addData("Цель", t == null ? "не видно" :
                    String.format("%s %.0f\" bearing %.1f° (%d%%)", t.name, t.distance, t.bearing, t.percentFound));
            telemetry.addData("Ошибка / допуск", "%.1f° / %.1f°", r.lastAimError, r.lastAimTolerance);
            telemetry.addData("RPM цель / L / R", "%.0f / %.0f / %.0f",
                    r.shooter.targetRpm(), r.shooter.rpmLeft(), r.shooter.rpmRight());
            telemetry.addData("Поправка RPM", Shooter.rpmTrim);
            telemetry.update();
        }
        r.stopAll();
        r.vision.close();
    }
}
