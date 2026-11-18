package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * AUTO 30 с (10.4). Робот всё делает сам:
 *  1) съезжает со стены на точку выстрела (LEAVE, 3 очка);
 *  2) наводится по AprilTag и стреляет, пока не вылетят 4 шара (считаются по просадке маховика);
 *  3) если есть время — собирает POLLEN и стреляет снова;
 *  4) паркуется в LOADING ZONE (PARK, 5 очков) и полностью останавливается (G403).
 * Маршрут — точки в Config (для красных; для синих зеркалятся). Держимся своей половины (G402).
 *
 * Init: X — синий, B — красный; D-pad ← / → — старт у зрителей / дальний.
 */
@Autonomous(name = "BioBuzz Auto", preselectTeleOp = "BioBuzz TeleOp")
public class MainAuto extends LinearOpMode {
    private Robot r;
    private final ElapsedTime match = new ElapsedTime();

    @Override public void runOpMode() {
        r = new Robot(hardwareMap, new Pose(0, 0, 0));
        boolean far = false;

        while (opModeInInit()) {
            r.update();
            if (gamepad1.x) Robot.alliance = Robot.Alliance.BLUE;
            if (gamepad1.b) Robot.alliance = Robot.Alliance.RED;
            if (gamepad1.dpad_left) far = false;
            if (gamepad1.dpad_right) far = true;
            telemetry.addData("Альянс (X / B)", Robot.alliance);
            telemetry.addData("Старт (D-pad ← / →)", far ? "дальний" : "у зрителей");
            telemetry.addLine("G304: касаться стены, своя половина, ровно 4 POLLEN, не в LOADING ZONE");
            r.addTelemetry(telemetry);
            telemetry.update();
        }
        match.reset();
        Robot.Alliance a = Robot.alliance;
        Pose shootPose = (far ? Config.SHOOT_FAR : Config.SHOOT_NEAR).forAlliance(a);

        r.shooter.idle();
        go(shootPose, 0.7, 2.5);                                         // LEAVE
        shoot(4, Config.AUTO_SHOOT1_DEADLINE);

        if (opModeIsActive() && match.seconds() < Config.AUTO_CYCLE_IF_BEFORE) {
            r.intake(1);
            go(Config.COLLECT_START.forAlliance(a), 0.7, 2.0);
            go(Config.COLLECT_END.forAlliance(a), 0.35, 2.5);           // медленно — подбираем
            r.intake(0.3);                                               // придерживаем
            go(shootPose, 0.7, 2.5);
            r.intake(0);
            shoot(4, Config.AUTO_SHOOT2_DEADLINE);
        }

        r.shooter.stop();
        go(Config.PARK.forAlliance(a), 0.8, 3.0);                        // PARK
        r.stopAll();
        Robot.lastPose = r.pose();
        while (opModeIsActive()) { r.update(); idle(); }                 // стоим до конца AUTO
        r.vision.close();
    }

    /** Ехать к точке, пока не приедем или не выйдет время. */
    private void go(Pose p, double power, double timeoutSec) {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < timeoutSec) {
            r.update();
            if (r.driveTo(p, power)) break;
            telemetry.addData("Еду к", p);
            r.addTelemetry(telemetry);
            telemetry.update();
        }
        r.stopDrive();
    }

    /** Наводиться и стрелять, пока не вылетит n шаров, бункер не опустеет или не наступит дедлайн. */
    private void shoot(int n, double deadlineSec) {
        int start = r.shooter.confirmedShots();
        r.shooter.resetEmptyKicks();
        while (opModeIsActive() && match.seconds() < deadlineSec) {
            r.update();
            int done = r.shooter.confirmedShots() - start;
            if (done >= n) break;
            if (r.shooter.emptyKicksInRow() >= Config.EMPTY_KICKS_TO_STOP) break;
            if (r.shooter.hasLoadedSensor() && !r.shooter.loaded() && !r.shooter.busy()) {
                r.intake(0.6);  // подтолкнуть следующий шар к толкателю
            } else {
                r.intake(0);
            }
            if (r.aim(0, 0, true, 0.18)) r.shooter.fire();
            telemetry.addData("Выстрелов", "%d / %d", done, n);
            telemetry.addData("Время", "%.1f с", match.seconds());
            r.addTelemetry(telemetry);
            telemetry.update();
        }
        while (opModeIsActive() && r.shooter.busy()) { r.update(); r.stopDrive(); }
        r.intake(0);
        r.stopDrive();
        r.shooter.idle();
    }
}
