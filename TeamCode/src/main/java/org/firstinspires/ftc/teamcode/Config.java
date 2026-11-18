package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/**
 * ВСЕ настройки робота в одном файле. Меняйте значения здесь, а не по коду.
 * Единицы: дюймы, градусы, миллисекунды, RPM — если не сказано иное.
 */
public final class Config {
    private Config() {}

    // ================= Имена устройств (как в конфигурации Driver Hub) =================
    public static String FL = "fl", FR = "fr", BL = "bl", BR = "br";
    public static String INTAKE = "intake";
    public static String SHOOTER_L = "shooterL", SHOOTER_R = "shooterR";
    public static String HOOD = "hood", KICKER = "kicker";
    public static String IMU = "imu";
    public static String PINPOINT = "pinpoint";          // необязательно
    public static String CAMERA = "Webcam 1";
    public static String LOADED_SENSOR = "loadedSensor"; // необязательно: шар у толкателя
    public static String FULL_SENSOR = "fullSensor";     // необязательно: 4-й шар в бункере

    // ================= Шасси =================
    public static double WHEEL_TICKS_PER_REV = 537.7;   // goBILDA 312 rpm
    public static double WHEEL_DIAMETER_IN = 4.09;      // 104 мм
    public static double STRAFE_MULT = 1.1;             // mecanum проскальзывает вбок
    public static RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection HUB_USB =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    // ================= goBILDA Pinpoint (если стоит) =================
    public static double PINPOINT_X_OFFSET_MM = -84;    // смещение боковой (X) колёсной ноги
    public static double PINPOINT_Y_OFFSET_MM = -168;   // смещение продольной (Y) ноги
    public static boolean PINPOINT_4_BAR_POD = true;    // false — swingarm pod
    public static boolean PINPOINT_X_REVERSED = false, PINPOINT_Y_REVERSED = false;

    // ================= Езда к точке (автономка и автоезда в TeleOp) =================
    public static double DRIVE_KP = 0.07;      // мощность на дюйм ошибки
    public static double DRIVE_KS = 0.07;      // минимум, чтобы робот тронулся
    public static double HEADING_KP = 1.6;     // мощность на радиан
    public static double POS_TOL_IN = 1.0, HEADING_TOL_DEG = 2.0;

    // ================= Автонаведение =================
    public static double AIM_KP = 1.4;         // мощность на радиан ошибки
    public static double AIM_KD = 0.12;        // гасит раскачку (по угловой скорости)
    public static double AIM_KS = 0.05;        // минимальная мощность поворота
    public static double AIM_MAX_TURN = 0.5;
    public static double AIM_OFFSET_DEG = 0.0; // камера стоит не по оси маховика
    public static double CELL_HALF_WIDTH_IN = 10;    // ячейка HIVE ~20" (9.6)
    public static double AIM_TOL_FRACTION = 0.4;     // попадать в центральные 40% ширины
    public static double AIM_MIN_TOL_DEG = 0.8;
    public static double AIM_MAX_TURN_RATE_DEG_S = 15; // стрелять только когда не крутится
    public static double READY_HOLD_MS = 80;
    public static double MIN_RANGE_IN = 30, MAX_RANGE_IN = 100, IDEAL_RANGE_IN = 60;
    public static double RANGE_KP = 0.03;

    // ================= Маховик =================
    public static double FLYWHEEL_TICKS_PER_REV = 28;   // goBILDA 6000 rpm 1:1
    // ТАБЛИЦА ВЫСТРЕЛА — заполните через OpMode "Shooter Tuner"
    public static double[] DIST_IN = {30,   42,   54,   66,   78,   90,   100};
    public static double[] RPM     = {2600, 2850, 3100, 3350, 3600, 3850, 4050};
    public static double[] HOOD_POS = {0.20, 0.28, 0.35, 0.42, 0.48, 0.53, 0.57};
    public static double IDLE_RPM = 2200;
    public static double SPEED_TOLERANCE = 0.025;       // ±2.5%
    public static double FLY_P = 25, FLY_I = 0, FLY_D = 0, FLY_F = 12;
    public static double NOMINAL_VOLTAGE = 12.5;        // при нём подобран FLY_F
    public static double SHOT_DROP = 0.04;              // просадка 4% = шар реально вылетел
    public static double SHOT_WINDOW_MS = 300;

    // ================= Толкатель =================
    public static double KICK_REST = 0.10, KICK_FIRE = 0.48;
    public static double KICK_OUT_MS = 140, KICK_BACK_MS = 200;
    public static int EMPTY_KICKS_TO_STOP = 2;          // в AUTO: 2 пустых толчка — бункер пуст

    // ================= Датчики бункера (REV Color/Distance V3 и т.п.) =================
    public static double LOADED_CM = 4.0, FULL_CM = 4.0;

    // ================= Камера =================
    public static Vision.Preference TARGET = Vision.Preference.NEAREST;
    public static double VISION_SMOOTH = 0.5;
    public static long VISION_MAX_AGE_MS = 300;
    public static float DECIMATION = 2;

    // ================= Матч =================
    public static double TELEOP_SEC = 120, ENDGAME_SEC = 60, PARK_WARN_SEC = 15;

    // ================= Точки маршрута (для КРАСНЫХ; для синих зеркалятся) =================
    // Система координат: старт робота = (0,0,0°); x — вперёд, y — влево, угол против часовой.
    public static Pose SHOOT_NEAR = Pose.deg(14, 0, 35);
    public static Pose SHOOT_FAR = Pose.deg(14, 0, -35);
    public static Pose COLLECT_START = Pose.deg(14, 18, 0);
    public static Pose COLLECT_END = Pose.deg(34, 18, 0);
    public static Pose PARK = Pose.deg(4, -24, 0);
    public static double AUTO_SHOOT1_DEADLINE = 12, AUTO_CYCLE_IF_BEFORE = 14, AUTO_SHOOT2_DEADLINE = 24;
}
