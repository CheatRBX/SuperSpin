package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Маховик на двух моторах (goBILDA 5203 6000 rpm, 28 тиков/оборот), серво-капюшон и серво-толкатель.
 * Обороты и капюшон берутся из таблицы по дистанции с линейной интерполяцией.
 * Таблицу заполняйте с помощью OpMode "Shooter Tuner".
 */
public class Shooter {
    public static final double TICKS_PER_REV = 28;

    // ======= ТАБЛИЦА ВЫСТРЕЛА (ЗАМЕНИТЕ на значения из Shooter Tuner) =======
    static final double[] DIST_IN = {30,   42,   54,   66,   78,   90,   100};
    static final double[] RPM     = {2600, 2850, 3100, 3350, 3600, 3850, 4050};
    static final double[] HOOD    = {0.20, 0.28, 0.35, 0.42, 0.48, 0.53, 0.57};
    // =======================================================================

    public static double IDLE_RPM = 2200;       // предраскрутка — быстрее первый выстрел
    public static double SPEED_TOLERANCE = 0.025; // ±2.5% от цели
    public static double rpmTrim = 0;           // поправка с джойстика (износ шаров/батарея)

    public static double KICK_REST = 0.10, KICK_FIRE = 0.48;
    public static double KICK_OUT_MS = 140, KICK_BACK_MS = 220;

    // PIDF скорости: F ≈ 32767 / макс. тиков в сек (~2800)
    public static double P = 25, I = 0, D = 0, F = 12;

    private final DcMotorEx left, right;
    public final Servo hood, kicker;
    private double targetRpm = 0;

    private enum Kick { IDLE, OUT, BACK }
    private Kick kick = Kick.IDLE;
    private final ElapsedTime kickTimer = new ElapsedTime();

    public Shooter(HardwareMap hw) {
        left = hw.get(DcMotorEx.class, "shooterL");
        right = hw.get(DcMotorEx.class, "shooterR");
        hood = hw.get(Servo.class, "hood");
        kicker = hw.get(Servo.class, "kicker");

        right.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotorEx m : new DcMotorEx[]{left, right}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setVelocityPIDFCoefficients(P, I, D, F);
        }
        kicker.setPosition(KICK_REST);
    }

    /** Обороты и капюшон по дистанции до цели (дюймы). */
    public void setDistance(double inches) {
        spin(interp(DIST_IN, RPM, inches) + rpmTrim);
        hood.setPosition(interp(DIST_IN, HOOD, inches));
    }

    public void spin(double rpm) {
        targetRpm = Math.max(0, rpm);
        double tps = targetRpm * TICKS_PER_REV / 60.0;
        left.setVelocity(tps);
        right.setVelocity(tps);
    }

    public void idle() { spin(IDLE_RPM); }

    public void stop() {
        targetRpm = 0;
        left.setPower(0);
        right.setPower(0);
    }

    public double targetRpm() { return targetRpm; }
    public double rpmLeft()  { return left.getVelocity() * 60.0 / TICKS_PER_REV; }
    public double rpmRight() { return right.getVelocity() * 60.0 / TICKS_PER_REV; }

    /** Оба мотора в пределах допуска — иначе выстрел уйдёт мимо. */
    public boolean atSpeed() {
        if (targetRpm < 500) return false;
        double tol = targetRpm * SPEED_TOLERANCE;
        return Math.abs(rpmLeft() - targetRpm) < tol && Math.abs(rpmRight() - targetRpm) < tol;
    }

    /** Толкнуть один шар. Возвращает false, если толкатель ещё не вернулся. */
    public boolean fire() {
        if (kick != Kick.IDLE) return false;
        kicker.setPosition(KICK_FIRE);
        kick = Kick.OUT;
        kickTimer.reset();
        return true;
    }

    public boolean busy() { return kick != Kick.IDLE; }

    /** Вызывать каждый цикл. */
    public void update() {
        if (kick == Kick.OUT && kickTimer.milliseconds() > KICK_OUT_MS) {
            kicker.setPosition(KICK_REST);
            kick = Kick.BACK;
            kickTimer.reset();
        } else if (kick == Kick.BACK && kickTimer.milliseconds() > KICK_BACK_MS) {
            kick = Kick.IDLE;
        }
    }

    static double interp(double[] xs, double[] ys, double x) {
        if (x <= xs[0]) return ys[0];
        for (int i = 1; i < xs.length; i++) {
            if (x <= xs[i]) {
                double k = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
                return ys[i - 1] + k * (ys[i] - ys[i - 1]);
            }
        }
        return ys[ys.length - 1];
    }
}
