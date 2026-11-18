package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

/**
 * Робот-стрелок BIOBUZZ. Каждый цикл вызывайте {@link #update()} — он читает все датчики
 * одним bulk-запросом, обновляет одометрию, камеру и маховик.
 * Моторов 7 (лимит 8), серво 2 (лимит 8) — R501.
 */
public class Robot {
    public enum Alliance { RED, BLUE }

    /** Выбирается в init и переходит из AUTO в TeleOp вместе с положением. */
    public static Alliance alliance = Alliance.RED;
    public static Pose lastPose = new Pose(0, 0, 0);

    public final DcMotorEx fl, fr, bl, br, intake;
    public final Shooter shooter;
    public final Vision vision;
    public final Localizer localizer;

    private final List<LynxModule> hubs;
    private final VoltageSensor battery;
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private double voltage = 12.5;

    private Vision.Target target = null;

    // история курса, чтобы учесть задержку камеры
    private static final int HIST = 64;
    private final long[] histNs = new long[HIST];
    private final double[] histH = new double[HIST];
    private int histIdx = 0;

    // наведение
    private double aimHeading = 0;
    private boolean aimValid = false;
    private boolean wasReady = false;
    private final ElapsedTime readyTimer = new ElapsedTime();
    public double aimErrorDeg = Double.NaN, aimTolDeg = Double.NaN;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private double loopMs = 0;

    public Robot(HardwareMap hw, Pose start) {
        hubs = hw.getAll(LynxModule.class);
        for (LynxModule h : hubs) h.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        battery = hw.voltageSensor.iterator().next();

        fl = hw.get(DcMotorEx.class, Config.FL);
        fr = hw.get(DcMotorEx.class, Config.FR);
        bl = hw.get(DcMotorEx.class, Config.BL);
        br = hw.get(DcMotorEx.class, Config.BR);
        intake = hw.get(DcMotorEx.class, Config.INTAKE);

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        GoBildaPinpointDriver pp = hw.tryGet(GoBildaPinpointDriver.class, Config.PINPOINT);
        if (pp != null) {
            localizer = new PinpointLocalizer(pp);
        } else {
            IMU imu = hw.get(IMU.class, Config.IMU);
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(Config.HUB_LOGO, Config.HUB_USB)));
            imu.resetYaw();
            localizer = new WheelLocalizer(fl, fr, bl, br, imu);
        }
        localizer.setPose(start);

        shooter = new Shooter(hw);
        vision = new Vision(hw);
        update();
    }

    // ============================== цикл ==============================

    /** Один раз в начале каждого цикла. */
    public void update() {
        for (LynxModule h : hubs) h.clearBulkCache();
        localizer.update();

        histNs[histIdx] = System.nanoTime();
        histH[histIdx] = localizer.getPose().h;
        histIdx = (histIdx + 1) % HIST;

        if (voltageTimer.milliseconds() > 500) {
            voltage = 0.7 * voltage + 0.3 * battery.getVoltage();
            shooter.setVoltage(voltage);
            voltageTimer.reset();
        }
        target = vision.update(alliance);
        shooter.update();

        loopMs = loopTimer.milliseconds();
        loopTimer.reset();
    }

    public Pose pose() { return localizer.getPose(); }
    public double heading() { return localizer.getPose().h; }
    public Vision.Target target() { return target; }
    public double voltage() { return voltage; }

    /** Обнулить курс для field-centric (робот смотрит «вперёд» от водителя). */
    public void resetHeading() {
        Pose p = pose();
        localizer.setPose(new Pose(p.x, p.y, 0));
    }

    /** Курс робота в момент снимка камеры. */
    private double headingAt(long ns) {
        double best = heading();
        long bestDt = Long.MAX_VALUE;
        for (int i = 0; i < HIST; i++) {
            long dt = Math.abs(histNs[i] - ns);
            if (histNs[i] != 0 && dt < bestDt) { bestDt = dt; best = histH[i]; }
        }
        return bestDt < 300_000_000L ? best : heading(); // старше 0.3 с — не верим
    }

    // ============================== движение ==============================

    /** Mecanum: y вперёд, x вправо, r > 0 — по часовой. */
    public void drive(double y, double x, double r, boolean fieldCentric) {
        if (fieldCentric) {
            double h = -heading();
            double nx = x * Math.cos(h) - y * Math.sin(h);
            double ny = x * Math.sin(h) + y * Math.cos(h);
            x = nx * Config.STRAFE_MULT;
            y = ny;
        }
        double d = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);
        fl.setPower((y + x + r) / d);
        bl.setPower((y - x + r) / d);
        fr.setPower((y - x - r) / d);
        br.setPower((y + x - r) / d);
    }

    public void stopDrive() { drive(0, 0, 0, false); }

    /** G403/G404: после AUTO и после матча робот неподвижен. */
    public void stopAll() {
        stopDrive();
        intake.setPower(0);
        shooter.stop();
    }

    /** Интейк с защитой от 5-го шара (G407): при полном бункере внутрь не тянет. */
    public void intake(double power) {
        if (power > 0 && shooter.hopperFull()) power = 0;
        intake.setPower(power);
    }

    /**
     * Один шаг езды к точке (P-регулятор + минимальная мощность).
     * @return true — приехали с точностью Config.POS_TOL_IN и HEADING_TOL_DEG
     */
    public boolean driveTo(Pose goal, double maxPower) {
        Pose p = pose();
        double dx = goal.x - p.x, dy = goal.y - p.y;
        double dist = Math.hypot(dx, dy);
        double fwd = dx * Math.cos(p.h) + dy * Math.sin(p.h);     // в системе робота
        double left = -dx * Math.sin(p.h) + dy * Math.cos(p.h);
        double herr = Pose.wrap(goal.h - p.h);

        double vf = fwd * Config.DRIVE_KP, vl = left * Config.DRIVE_KP;
        double mag = Math.hypot(vf, vl);
        if (mag > maxPower) { vf *= maxPower / mag; vl *= maxPower / mag; mag = maxPower; }
        if (dist > Config.POS_TOL_IN && mag < Config.DRIVE_KS && mag > 1e-6) {
            vf *= Config.DRIVE_KS / mag; vl *= Config.DRIVE_KS / mag;
        }
        double turn = Range.clip(herr * Config.HEADING_KP, -0.6, 0.6);
        drive(vf, -vl, -turn, false);

        return dist < Config.POS_TOL_IN && Math.abs(herr) < Math.toRadians(Config.HEADING_TOL_DEG);
    }

    // ============================== наведение ==============================

    public boolean aim(double y, double x, boolean approach, double searchTurn) {
        return aim(y, x, approach, searchTurn, true);
    }

    /**
     * Один шаг автонаведения. Камера даёт угол на цель, он переводится в абсолютный курс
     * (с учётом курса в момент снимка), а дальше робот держит этот курс по одометрии —
     * без раскачки от задержки камеры.
     *
     * @param y,x           движение от водителя (в AUTO 0)
     * @param approach      самому подъехать в рабочую дистанцию
     * @param searchTurn    мощность поиска, если цели не видно (0 — не искать)
     * @param controlShooter ставить обороты/капюшон по дистанции
     * @return true — можно стрелять: навёлся, не крутится, дистанция ок, маховик на скорости, шар готов
     */
    public boolean aim(double y, double x, boolean approach, double searchTurn, boolean controlShooter) {
        Vision.Target t = target;
        if (t == null) {
            aimValid = false;
            aimErrorDeg = Double.NaN;
            wasReady = false;
            drive(y, x, searchTurn, false);
            return false;
        }
        if (t.fresh || !aimValid) {
            aimHeading = Pose.wrap(headingAt(t.frameNs) + Math.toRadians(t.bearing - Config.AIM_OFFSET_DEG));
            aimValid = true;
        }
        if (controlShooter) shooter.setDistance(t.distance);

        double err = Pose.wrap(aimHeading - heading());      // >0 — повернуть влево
        double hv = localizer.headingVelocity();
        double tol = Math.max(Math.toRadians(Config.AIM_MIN_TOL_DEG),
                Math.atan(Config.CELL_HALF_WIDTH_IN * Config.AIM_TOL_FRACTION / t.distance));
        aimErrorDeg = Math.toDegrees(err);
        aimTolDeg = Math.toDegrees(tol);

        double out = Config.AIM_KP * err - Config.AIM_KD * hv;
        if (Math.abs(err) > tol * 0.5) out += Math.copySign(Config.AIM_KS, err);
        out = Range.clip(out, -Config.AIM_MAX_TURN, Config.AIM_MAX_TURN);

        boolean inRange = t.distance >= Config.MIN_RANGE_IN && t.distance <= Config.MAX_RANGE_IN;
        if (approach && !inRange) {
            y = Range.clip((t.distance - Config.IDEAL_RANGE_IN) * Config.RANGE_KP, -0.5, 0.5);
        }
        drive(y, x, -out, false);

        boolean still = Math.abs(y) < 0.1 && Math.abs(x) < 0.1;
        boolean steady = Math.abs(hv) < Math.toRadians(Config.AIM_MAX_TURN_RATE_DEG_S);
        boolean ready = Math.abs(err) < tol && steady && inRange && still && shooter.ready();
        if (ready && !wasReady) readyTimer.reset();
        wasReady = ready;
        return ready && readyTimer.milliseconds() >= Config.READY_HOLD_MS;
    }

    // ============================== телеметрия ==============================

    public void addTelemetry(Telemetry tm) {
        Vision.Target t = target;
        tm.addData("Альянс", alliance);
        tm.addData("Позиция (" + localizer.name() + ")", pose());
        tm.addData("Цель", t == null ? "не видно"
                : String.format("%s  %.0f\"  %.1f°  (%d%%)", t.name, t.distance, t.bearing, t.percentFound));
        tm.addData("Наведение ошибка/допуск", "%.1f° / %.1f°", aimErrorDeg, aimTolDeg);
        tm.addData("Маховик цель / L / R", "%.0f / %.0f / %.0f  (поправка %+.0f)",
                shooter.targetRpm(), shooter.rpmLeft(), shooter.rpmRight(), Shooter.rpmTrim);
        tm.addData("Выстрелов подтверждено", shooter.confirmedShots());
        tm.addData("Бункер", (shooter.hasLoadedSensor() ? (shooter.loaded() ? "шар готов" : "пусто") : "датчика нет")
                + (shooter.hasFullSensor() && shooter.hopperFull() ? " | ПОЛОН (4)" : ""));
        tm.addData("Батарея / цикл / камера", "%.2f В / %.0f мс / %.0f fps", voltage, loopMs, vision.fps());
    }
}
