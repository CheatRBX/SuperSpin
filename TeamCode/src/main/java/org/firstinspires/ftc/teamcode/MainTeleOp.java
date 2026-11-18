package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Gamepad1 (водитель)
 *   левый стик — езда, правый стик X — поворот, RT — медленно
 *   RB (держать) — автонаведение + автовыстрел
 *   LB (держать) — то же + сам подъезжает на рабочую дистанцию
 *   Y  (держать) — автоезда к сохранённой точке выстрела,  D-pad ← — сохранить точку здесь
 *   A  (держать) — автоезда в LOADING ZONE (парковка)
 *   BACK — field/robot-centric,  START — обнулить курс
 * Gamepad2 (оператор)
 *   RT — интейк внутрь, LT — наружу,  X — ручной выстрел
 *   Y — предраскрутка вкл/выкл,  B — маховик стоп,  A (держать) — выстрел без камеры (54")
 *   D-pad ↑/↓ — поправка оборотов ±50
 * Init: X — синий альянс, B — красный.
 * Подсветка Gamepad1: зелёный — огонь, жёлтый — наводится, красный — цели нет.
 * Вибрация: бункер полон; 60 с до конца (ENDGAME); 15 с — пора парковаться.
 */
@TeleOp(name = "BioBuzz TeleOp")
public class MainTeleOp extends LinearOpMode {
    private int led = -1;

    @Override public void runOpMode() {
        Robot r = new Robot(hardwareMap, Robot.lastPose);
        Pose shotSpot = null;
        boolean field = true, prespin = true, wasFull = false, endgameBuzz = false, parkBuzz = false;

        while (opModeInInit()) {
            r.update();
            if (gamepad1.x || gamepad2.x) Robot.alliance = Robot.Alliance.BLUE;
            if (gamepad1.b || gamepad2.b) Robot.alliance = Robot.Alliance.RED;
            telemetry.addLine("X — синий, B — красный");
            r.addTelemetry(telemetry);
            telemetry.update();
        }
        ElapsedTime match = new ElapsedTime();

        while (opModeIsActive()) {
            r.update();
            double left = Config.TELEOP_SEC - match.seconds();

            // ---------- переключатели ----------
            if (gamepad1.backWasPressed()) field = !field;
            if (gamepad1.startWasPressed()) r.resetHeading();
            if (gamepad1.dpadLeftWasPressed()) { shotSpot = r.pose(); gamepad1.rumble(150); }
            if (gamepad2.yWasPressed()) prespin = !prespin;
            if (gamepad2.dpadUpWasPressed()) Shooter.rpmTrim += 50;
            if (gamepad2.dpadDownWasPressed()) Shooter.rpmTrim -= 50;

            // ---------- движение и стрельба ----------
            double slow = gamepad1.right_trigger > 0.3 ? 0.35 : 1.0;
            double y = -gamepad1.left_stick_y * slow, x = gamepad1.left_stick_x * slow;
            boolean aiming = gamepad1.right_bumper || gamepad1.left_bumper;
            boolean ready = false;
            String mode;

            if (aiming) {
                ready = r.aim(y, x, gamepad1.left_bumper, 0);
                if (ready) r.shooter.fire();
                mode = ready ? "ОГОНЬ" : "наведение";
            } else if (gamepad1.y && shotSpot != null) {
                r.driveTo(shotSpot, 0.8);
                r.shooter.idle();
                mode = "автоезда к точке выстрела";
            } else if (gamepad1.a) {
                r.driveTo(Config.PARK.forAlliance(Robot.alliance), 0.8);
                mode = "автопарковка";
            } else {
                r.drive(y, x, gamepad1.right_stick_x * slow, field);
                if (gamepad2.a) r.shooter.setDistance(54);
                else if (gamepad2.b || !prespin) r.shooter.stop();
                else r.shooter.idle();
                mode = field ? "field-centric" : "robot-centric";
            }

            if (gamepad2.x) r.shooter.fire();
            r.intake(gamepad2.right_trigger - gamepad2.left_trigger);

            // ---------- обратная связь ----------
            boolean full = r.shooter.hopperFull();
            if (full && !wasFull) { gamepad2.rumble(300); gamepad1.rumble(150); }
            wasFull = full;
            if (!endgameBuzz && left <= Config.ENDGAME_SEC) { endgameBuzz = true; buzz(gamepad1); buzz(gamepad2); }
            if (!parkBuzz && left <= Config.PARK_WARN_SEC) { parkBuzz = true; gamepad1.rumble(1, 1, 800); }

            int state = !aiming ? 0 : ready ? 1 : r.target() != null ? 2 : 3;
            setLed(state);

            telemetry.addData("Режим", mode);
            telemetry.addData("Осталось", "%.0f с%s", Math.max(0, left), left <= Config.ENDGAME_SEC ? "  (ENDGAME)" : "");
            telemetry.addData("Точка выстрела", shotSpot == null ? "не сохранена (D-pad ←)" : shotSpot.toString());
            r.addTelemetry(telemetry);
            telemetry.update();
        }
        r.stopAll();
        Robot.lastPose = r.pose();
        r.vision.close();
    }

    private void buzz(Gamepad g) { g.rumble(0.8, 0.8, 250); }

    private void setLed(int state) {
        if (state == led) return;
        led = state;
        int d = Gamepad.LED_DURATION_CONTINUOUS;
        switch (state) {
            case 1: gamepad1.setLedColor(0, 1, 0, d); break;
            case 2: gamepad1.setLedColor(1, 0.8, 0, d); break;
            case 3: gamepad1.setLedColor(1, 0, 0, d); break;
            default:
                if (Robot.alliance == Robot.Alliance.RED) gamepad1.setLedColor(1, 0, 0.1, d);
                else gamepad1.setLedColor(0, 0.2, 1, d);
        }
    }
}
