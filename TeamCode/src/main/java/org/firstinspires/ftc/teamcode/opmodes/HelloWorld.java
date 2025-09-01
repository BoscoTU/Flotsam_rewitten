package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class HelloWorld extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("I am waiting to be started");

        waitForStart();
        telemetry.addLine("Hello world");

        while(opModeIsActive() && !isStopRequested()) {
            telemetry.addData("left StickY", gamepad1.left_stick_y);
            telemetry.addData("left StickX", gamepad1.left_stick_x);
            telemetry.addData("right StickX",gamepad1.right_stick_x);
            telemetry.addData("button A", gamepad1.a);
            telemetry.update();
        }

        if(isStopRequested()) {
            telemetry.addData("Ended program", isStopRequested());
            telemetry.update();
        }
    }
}
