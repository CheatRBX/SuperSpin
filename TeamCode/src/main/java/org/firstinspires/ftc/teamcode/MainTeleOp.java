package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "BioBuzz TeleOp")
public class MainTeleOp extends LinearOpMode {
    @Override public void runOpMode() {
        Robot r = new Robot(hardwareMap);
        boolean field = true, clawOpen = false, lastA = false, lastBack = false;
        telemetry.addLine("Готов"); telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // --- Gamepad1: шасси ---
            double slow = gamepad1.right_trigger > 0.3 ? 0.35 : 1.0;
            if (gamepad1.back && !lastBack) field = !field;
            lastBack = gamepad1.back;
            if (gamepad1.start) r.imu.resetYaw();
            r.drive(-gamepad1.left_stick_y * slow, gamepad1.left_stick_x * slow,
                    gamepad1.right_stick_x * slow, field);

            // --- Gamepad2: механизмы ---
            r.intake.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

            if (gamepad2.a && !lastA) clawOpen = !clawOpen;
            lastA = gamepad2.a;
            r.claw.setPosition(clawOpen ? Robot.CLAW_OPEN : Robot.CLAW_CLOSED);

            if (gamepad2.dpad_down) r.liftTo(Robot.LIFT_MIN, 1);
            else if (gamepad2.dpad_left) r.liftTo(1200, 1);
            else if (gamepad2.dpad_up) r.liftTo(Robot.LIFT_MAX, 1);
            else if (Math.abs(gamepad2.left_stick_y) > 0.1) {
                int pos = r.lift.getCurrentPosition();
                double p = -gamepad2.left_stick_y;
                if ((pos <= Robot.LIFT_MIN && p < 0) || (pos >= Robot.LIFT_MAX && p > 0)) p = 0;
                r.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                r.lift.setPower(p);
            } else if (r.lift.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                r.liftTo(r.lift.getCurrentPosition(), 0.5); // удержание
            }

            telemetry.addData("Field-centric", field);
            telemetry.addData("Heading°", Math.toDegrees(r.heading()));
            telemetry.addData("Lift", r.lift.getCurrentPosition());
            telemetry.addData("Claw", clawOpen ? "open" : "closed");
            telemetry.update();
        }
    }
}
