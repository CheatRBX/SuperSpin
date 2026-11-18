package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * AUTO 30 с (10.4). Робот сам: съезжает со стены (LEAVE, 3 очка), наводится по AprilTag
 * и выстреливает 4 предзаряженных POLLEN, собирает ещё, стреляет снова и паркуется
 * в LOADING ZONE (PARK, 5 очков). Держится своей половины поля (G402).
 *
 * Init: X — синий, B — красный; D-pad влево/вправо — старт у зрителей / дальний.
 * ВСЕ дистанции и углы ниже — заготовка: промерьте на своём поле.
 * Для синего альянса боковые движения и повороты зеркалятся.
 */
@Autonomous(name = "BioBuzz Auto", preselectTeleOp = "BioBuzz TeleOp")
public class MainAuto extends LinearOpMode {
    // --- маршрут (дюймы / градусы, для КРАСНОГО) ---
    static double LEAVE_FWD = 14;
    static double AIM_HEADING_NEAR = 35, AIM_HEADING_FAR = -35; // примерно на HIVE
    static double COLLECT_STRAFE = 18, COLLECT_FWD = 20;
    static double PARK_STRAFE = -24, PARK_FWD = -10;
    // --- время ---
    static double SHOOT1_DEADLINE = 12, CYCLE_IF_BEFORE = 14, SHOOT2_DEADLINE = 24;

    Robot r;
    final ElapsedTime match = new ElapsedTime();
    boolean farStart = false;

    @Override public void runOpMode() {
        r = new Robot(hardwareMap);

        while (opModeInInit()) {
            if (gamepad1.x) Robot.alliance = Robot.Alliance.BLUE;
            if (gamepad1.b) Robot.alliance = Robot.Alliance.RED;
            if (gamepad1.dpad_left) farStart = false;
            if (gamepad1.dpad_right) farStart = true;
            Vision.Target t = r.vision.update(Robot.alliance);
            telemetry.addData("Альянс (X/B)", Robot.alliance);
            telemetry.addData("Старт (D-pad ←/→)", farStart ? "дальний" : "у зрителей");
            telemetry.addData("HIVE видно", t == null ? "нет" : String.format("%.0f\"", t.distance));
            telemetry.addLine("Робот: касается стены, на своей половине, ровно 4 POLLEN (G304)");
            telemetry.update();
        }
        match.reset();
        double side = Robot.alliance == Robot.Alliance.RED ? 1 : -1;

        r.shooter.idle();
        move(LEAVE_FWD, 0, 0.6, 2.0);                                     // LEAVE
        turnTo(side * (farStart ? AIM_HEADING_FAR : AIM_HEADING_NEAR), 0.5, 1.5);
        shoot(4, SHOOT1_DEADLINE);

        if (match.seconds() < CYCLE_IF_BEFORE) {                          // второй цикл
            r.intake.setPower(1);
            turnTo(0, 0.5, 1.2);
            move(0, side * COLLECT_STRAFE, 0.6, 2.0);
            move(COLLECT_FWD, 0, 0.35, 2.5);                              // медленно — подбираем
            sleep(300);
            r.intake.setPower(0.3);                                       // придерживаем (не больше 4, G407)
            move(-COLLECT_FWD, 0, 0.6, 2.0);
            turnTo(side * (farStart ? AIM_HEADING_FAR : AIM_HEADING_NEAR), 0.5, 1.5);
            r.intake.setPower(0);
            shoot(4, SHOOT2_DEADLINE);
        }

        r.shooter.stop();
        turnTo(0, 0.5, 1.0);
        move(PARK_FWD, side * PARK_STRAFE, 0.7, 3.0);                     // PARK
        r.stopAll();                                                      // G403: стоим в переходе
        r.vision.close();
    }

    /** Наводится и стреляет до n раз, пока не наступит дедлайн. Цель ищет медленным поворотом. */
    void shoot(int n, double deadlineSec) {
        int shots = 0;
        ElapsedTime sinceLast = new ElapsedTime();
        while (opModeIsActive() && shots < n && match.seconds() < deadlineSec) {
            boolean ready = r.aimStep(0, 0, true, 0.18);
            if (ready && !r.shooter.busy() && sinceLast.milliseconds() > 250 && r.shooter.fire()) {
                shots++;
                sinceLast.reset();
            }
            r.shooter.update();
            telemetry.addData("Выстрелов", shots);
            telemetry.addData("Время", "%.1f", match.seconds());
            telemetry.update();
        }
        while (opModeIsActive() && r.shooter.busy()) r.shooter.update();
        r.stopDrive();
        r.shooter.idle();
    }

    /** Езда по энкодерам. fwd/strafe в дюймах, с таймаутом. */
    void move(double fwd, double strafe, double power, double timeoutSec) {
        DcMotorEx[] m = {r.fl, r.fr, r.bl, r.br};
        double[] d = {fwd + strafe, fwd - strafe, fwd - strafe, fwd + strafe};
        for (int i = 0; i < 4; i++) {
            m[i].setTargetPosition(m[i].getCurrentPosition() + (int) (d[i] * Robot.TICKS_PER_INCH));
            m[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m[i].setPower(power);
        }
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < timeoutSec
                && (r.fl.isBusy() || r.fr.isBusy() || r.bl.isBusy() || r.br.isBusy())) {
            r.shooter.update();
        }
        for (DcMotorEx x : m) { x.setPower(0); x.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); }
    }

    /** Поворот на курс (градусы, против часовой +) по IMU. */
    void turnTo(double deg, double maxPower, double timeoutSec) {
        double target = Math.toRadians(deg);
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < timeoutSec) {
            double e = target - r.heading();
            e = Math.atan2(Math.sin(e), Math.cos(e));
            if (Math.abs(e) < Math.toRadians(1.5)) break;
            double p = Math.max(-maxPower, Math.min(maxPower, e * 1.2));
            if (Math.abs(p) < 0.08) p = Math.copySign(0.08, p);
            r.drive(0, 0, -p, false);
            r.shooter.update();
        }
        r.stopDrive();
    }
}
