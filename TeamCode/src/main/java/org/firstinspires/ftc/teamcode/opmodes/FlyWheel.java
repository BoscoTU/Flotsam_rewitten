package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Test Motor Control", group = "Examples")
public class FlyWheel extends LinearOpMode {
//http://192.168.43.1:8080/?page=connection.html&pop=true
    @Override
    public void runOpMode() {
        DcMotorEx testMotor = hardwareMap.get(DcMotorEx.class, "test");

        // Optional: set to run with encoder if you want velocity feedback
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double power = 0.0;
        double step = 0.05; // how much to increase/decrease per button press

        waitForStart();

        while (opModeIsActive()) {
            // Increase power
            if (gamepad1.dpad_up) {
                power += step;
            }
            // Decrease power
            if (gamepad1.dpad_down) {
                power -= step;
            }

            // Clip power to safe range
            power = Math.max(-1.0, Math.min(1.0, power));
            testMotor.setPower(power);

            // Get velocity (ticks per second)
            double velocityTicksPerSec = testMotor.getVelocity();
            // Convert to RPM: (ticks/sec) * (60 / ticksPerRev)
            // Replace 28 with your motor’s ticks per revolution (for example, NeveRest 40 = 1120)
            double ticksPerRev = 341;
            double rpm = (velocityTicksPerSec * 60.0) / ticksPerRev;

            telemetry.addData("Motor Power", "%.2f", power);
            telemetry.addData("Velocity (ticks/s)", "%.2f", velocityTicksPerSec);
            telemetry.addData("Velocity (RPM)", "%.2f", rpm);
            telemetry.update();

            sleep(100); // small delay to avoid spamming
        }
    }
}
