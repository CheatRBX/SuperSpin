package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Маховик на двух моторах + серво-капюшон + серво-толкатель.
 *  - обороты и капюшон по дистанции из таблицы (Config.DIST_IN / RPM / HOOD_POS);
 *  - компенсация напряжения батареи (feedforward F масштабируется);
 *  - подтверждение выстрела по просадке оборотов: считаем только реально вылетевшие шары;
 *  - необязательные датчики: шар у толкателя / бункер полон (лимит 4 по G407).
 */
public class Shooter {
    private final DcMotorEx left, right;
    public final Servo hood, kicker;
    private final DistanceSensor loadedSensor, fullSensor; // могут быть null

    public static double rpmTrim = 0;   // поправка с джойстика

    private double targetRpm = 0, appliedVoltage = 0;

    private enum Kick { IDLE, OUT, BACK }
    private Kick kick = Kick.IDLE;
    private final ElapsedTime kickTimer = new ElapsedTime();

    // подтверждение выстрела
    private boolean watching = false;
    private double watchTargetRpm = 0, watchMinRpm = 0;
    private final ElapsedTime watchTimer = new ElapsedTime();
    private int confirmedShots = 0, emptyKicksInRow = 0;

    public Shooter(HardwareMap hw) {
        left = hw.get(DcMotorEx.class, Config.SHOOTER_L);
        right = hw.get(DcMotorEx.class, Config.SHOOTER_R);
        hood = hw.get(Servo.class, Config.HOOD);
        kicker = hw.get(Servo.class, Config.KICKER);
        loadedSensor = hw.tryGet(DistanceSensor.class, Config.LOADED_SENSOR);
        fullSensor = hw.tryGet(DistanceSensor.class, Config.FULL_SENSOR);

        right.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotorEx m : new DcMotorEx[]{left, right}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        setVoltage(Config.NOMINAL_VOLTAGE);
        kicker.setPosition(Config.KICK_REST);
    }

    /** Перенастраивает F под текущее напряжение, чтобы обороты не плыли с разрядом батареи. */
    public void setVoltage(double v) {
        if (v < 7 || Math.abs(v - appliedVoltage) < 0.25) return;
        appliedVoltage = v;
        double f = Config.FLY_F * Config.NOMINAL_VOLTAGE / v;
        left.setVelocityPIDFCoefficients(Config.FLY_P, Config.FLY_I, Config.FLY_D, f);
        right.setVelocityPIDFCoefficients(Config.FLY_P, Config.FLY_I, Config.FLY_D, f);
    }

    /** Обороты и капюшон по дистанции до цели (дюймы). */
    public void setDistance(double inches) {
        spin(interp(Config.DIST_IN, Config.RPM, inches) + rpmTrim);
        hood.setPosition(interp(Config.DIST_IN, Config.HOOD_POS, inches));
    }

    public void spin(double rpm) {
        targetRpm = Math.max(0, rpm);
        double tps = targetRpm * Config.FLYWHEEL_TICKS_PER_REV / 60.0;
        left.setVelocity(tps);
        right.setVelocity(tps);
    }

    public void idle() { spin(Config.IDLE_RPM); }

    public void stop() {
        targetRpm = 0;
        left.setPower(0);
        right.setPower(0);
    }

    public double targetRpm() { return targetRpm; }
    public double rpmLeft()  { return left.getVelocity() * 60.0 / Config.FLYWHEEL_TICKS_PER_REV; }
    public double rpmRight() { return right.getVelocity() * 60.0 / Config.FLYWHEEL_TICKS_PER_REV; }

    public boolean atSpeed() {
        if (targetRpm < 500) return false;
        double tol = targetRpm * Config.SPEED_TOLERANCE;
        return Math.abs(rpmLeft() - targetRpm) < tol && Math.abs(rpmRight() - targetRpm) < tol;
    }

    public boolean hasLoadedSensor() { return loadedSensor != null; }
    public boolean hasFullSensor() { return fullSensor != null; }

    /** Шар у толкателя. Без датчика считаем, что есть. */
    public boolean loaded() {
        return loadedSensor == null || loadedSensor.getDistance(DistanceUnit.CM) < Config.LOADED_CM;
    }

    /** В бункере 4 шара — больше брать нельзя (G407). Без датчика — false. */
    public boolean hopperFull() {
        return fullSensor != null && fullSensor.getDistance(DistanceUnit.CM) < Config.FULL_CM;
    }

    /** Можно стрелять прямо сейчас. */
    public boolean ready() { return atSpeed() && loaded() && kick == Kick.IDLE; }

    /** Толкнуть один шар. false — толкатель ещё занят. */
    public boolean fire() {
        if (kick != Kick.IDLE) return false;
        kicker.setPosition(Config.KICK_FIRE);
        kick = Kick.OUT;
        kickTimer.reset();
        watching = targetRpm > 500;
        watchTargetRpm = targetRpm;
        watchMinRpm = Double.MAX_VALUE;
        watchTimer.reset();
        return true;
    }

    public boolean busy() { return kick != Kick.IDLE || watching; }
    public int confirmedShots() { return confirmedShots; }
    public int emptyKicksInRow() { return emptyKicksInRow; }
    public void resetEmptyKicks() { emptyKicksInRow = 0; }

    /** Вызывать каждый цикл (делает Robot.update()). */
    public void update() {
        if (kick == Kick.OUT && kickTimer.milliseconds() > Config.KICK_OUT_MS) {
            kicker.setPosition(Config.KICK_REST);
            kick = Kick.BACK;
            kickTimer.reset();
        } else if (kick == Kick.BACK && kickTimer.milliseconds() > Config.KICK_BACK_MS) {
            kick = Kick.IDLE;
        }

        if (watching) {
            watchMinRpm = Math.min(watchMinRpm, Math.min(rpmLeft(), rpmRight()));
            if ((watchTargetRpm - watchMinRpm) / watchTargetRpm > Config.SHOT_DROP) {
                confirmedShots++;
                emptyKicksInRow = 0;
                watching = false;
            } else if (watchTimer.milliseconds() > Config.SHOT_WINDOW_MS) {
                emptyKicksInRow++;
                watching = false;
            }
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
