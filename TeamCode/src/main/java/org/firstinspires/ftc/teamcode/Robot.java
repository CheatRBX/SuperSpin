package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Робот-стрелок для BIOBUZZ: mecanum-шасси, интейк, двухмоторный маховик, камера с AprilTag.
 * Моторов 7 (лимит 8, R501), серво 2 (лимит 8).
 * Имена устройств должны совпадать с конфигурацией на Driver Hub.
 */
public class Robot {
    public enum Alliance { RED, BLUE }

    /** Выбирается в init автономки и сохраняется для TeleOp. */
    public static Alliance alliance = Alliance.RED;

    // goBILDA 312 rpm (537.7 тиков/оборот), колесо 104 мм
    public static final double TICKS_PER_INCH = 537.7 / (Math.PI * 4.09);

    // --- Автонаведение (подстраивается) ---
    public static double AIM_KP = 0.022;        // мощность поворота на 1° ошибки
    public static double AIM_MIN_TURN = 0.06;   // чтобы дотянуть последние градусы
    public static double AIM_MAX_TURN = 0.45;
    public static double AIM_OFFSET_DEG = 0.0;  // если камера стоит не по оси маховика
    public static double MIN_RANGE_IN = 30, MAX_RANGE_IN = 100, IDEAL_RANGE_IN = 60;
    public static double RANGE_KP = 0.025;
    public static double CELL_HALF_WIDTH_IN = 10; // ячейка HIVE ~20" шириной
    public static double READY_HOLD_MS = 100;     // сколько держать "готов" перед выстрелом

    public final DcMotorEx fl, fr, bl, br, intake;
    public final IMU imu;
    public final Shooter shooter;
    public final Vision vision;

    private final ElapsedTime readyTimer = new ElapsedTime();
    private boolean wasReady = false;

    // для телеметрии
    public double lastAimError = Double.NaN, lastAimTolerance = Double.NaN;

    public Robot(HardwareMap hw) {
        fl = hw.get(DcMotorEx.class, "fl");
        fr = hw.get(DcMotorEx.class, "fr");
        bl = hw.get(DcMotorEx.class, "bl");
        br = hw.get(DcMotorEx.class, "br");
        intake = hw.get(DcMotorEx.class, "intake");
        imu = hw.get(IMU.class, "imu");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        shooter = new Shooter(hw);
        vision = new Vision(hw);
    }

    /** Курс в радианах, против часовой — положительный. */
    public double heading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /** Mecanum: y вперёд, x вправо, r > 0 — поворот по часовой. */
    public void drive(double y, double x, double r, boolean fieldCentric) {
        if (fieldCentric) {
            double h = -heading();
            double nx = x * Math.cos(h) - y * Math.sin(h);
            double ny = x * Math.sin(h) + y * Math.cos(h);
            x = nx * 1.1;
            y = ny;
        }
        double d = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);
        fl.setPower((y + x + r) / d);
        bl.setPower((y - x + r) / d);
        fr.setPower((y - x - r) / d);
        br.setPower((y + x - r) / d);
    }

    public void stopDrive() { drive(0, 0, 0, false); }

    /** Полная остановка — по G403/G404 робот должен стоять после AUTO и после матча. */
    public void stopAll() {
        stopDrive();
        intake.setPower(0);
        shooter.stop();
    }

    /**
     * Один шаг автонаведения: ищет HIVE своего альянса, поворачивает робота на цель,
     * ставит обороты маховика и угол капюшона по дистанции.
     *
     * @param y,x         движение от водителя (в автономке 0)
     * @param approach    самому подъехать в рабочую дистанцию
     * @param searchTurn  мощность поворота для поиска цели, если её не видно (0 — не искать)
     * @return true, когда робот навёлся, маховик на скорости и это держится READY_HOLD_MS
     */
    public boolean aimStep(double y, double x, boolean approach, double searchTurn) {
        Vision.Target t = vision.update(alliance);
        if (t == null) {
            lastAimError = Double.NaN;
            drive(y, x, searchTurn, false);
            wasReady = false;
            return false;
        }

        shooter.setDistance(t.distance);

        double err = t.bearing - AIM_OFFSET_DEG;          // >0 — цель левее
        // Допуск: шар должен попасть в центральные 40% ширины ячейки
        double tol = Math.max(0.8, Math.toDegrees(Math.atan(CELL_HALF_WIDTH_IN * 0.4 / t.distance)));
        lastAimError = err;
        lastAimTolerance = tol;

        double turn = 0;
        if (Math.abs(err) > tol * 0.5) {
            turn = -err * AIM_KP;
            if (Math.abs(turn) < AIM_MIN_TURN) turn = Math.copySign(AIM_MIN_TURN, turn);
            turn = Range.clip(turn, -AIM_MAX_TURN, AIM_MAX_TURN);
        }

        boolean inRange = t.distance >= MIN_RANGE_IN && t.distance <= MAX_RANGE_IN;
        if (approach && !inRange) {
            y = Range.clip((t.distance - IDEAL_RANGE_IN) * RANGE_KP, -0.5, 0.5);
        }
        drive(y, x, turn, false);

        boolean still = Math.abs(y) < 0.1 && Math.abs(x) < 0.1; // на ходу точность хуже
        boolean ready = Math.abs(err) < tol && inRange && still && shooter.atSpeed();
        if (ready && !wasReady) readyTimer.reset();
        wasReady = ready;
        return ready && readyTimer.milliseconds() >= READY_HOLD_MS;
    }
}
