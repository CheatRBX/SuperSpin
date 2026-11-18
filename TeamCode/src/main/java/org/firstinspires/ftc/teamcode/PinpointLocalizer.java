package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/** goBILDA Pinpoint: две пассивные колёсные ноги + свой IMU. Не проскальзывает как mecanum. */
public class PinpointLocalizer implements Localizer {
    private final GoBildaPinpointDriver pp;
    private Pose pose = new Pose(0, 0, 0);
    private double hv = 0;
    private Pose pending = null; // применим, когда Pinpoint закончит калибровку

    public PinpointLocalizer(GoBildaPinpointDriver pp) {
        this.pp = pp;
        pp.setOffsets(Config.PINPOINT_X_OFFSET_MM, Config.PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);
        pp.setEncoderResolution(Config.PINPOINT_4_BAR_POD
                ? GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
                : GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pp.setEncoderDirections(
                Config.PINPOINT_X_REVERSED ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                Config.PINPOINT_Y_REVERSED ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pp.resetPosAndIMU(); // робот должен стоять неподвижно ~0.25 с
    }

    @Override public void update() {
        pp.update();
        if (pending != null && pp.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {
            pp.setPosition(new Pose2D(DistanceUnit.INCH, pending.x, pending.y, AngleUnit.RADIANS, pending.h));
            pp.update();
            pending = null;
        }
        Pose2D p = pp.getPosition();
        pose = new Pose(p.getX(DistanceUnit.INCH), p.getY(DistanceUnit.INCH), p.getHeading(AngleUnit.RADIANS));
        hv = pp.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
    }

    @Override public Pose getPose() { return pose; }

    @Override public void setPose(Pose p) {
        pending = p;
        pose = p;
    }

    public GoBildaPinpointDriver.DeviceStatus status() { return pp.getDeviceStatus(); }

    @Override public double headingVelocity() { return hv; }
    @Override public String name() { return "Pinpoint " + pp.getDeviceStatus(); }
}
