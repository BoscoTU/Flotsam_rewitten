package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "elevator (Blocks to Java)")
public class Elevator extends LinearOpMode {
    private DigitalChannel touchSensor;
    private DcMotor test;

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {
        touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");
        test = hardwareMap.get(DcMotor.class, "test");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);


        // Put initialization blocks here.
        waitForStart();
        // Put run blocks here.
        while (opModeIsActive()) {
            if (touchSensor.getState() == false) {
                test.setPower(Range.clip(-1 * gamepad1.left_stick_y, -1, 0));
            } else {
                test.setPower(-1 * gamepad1.left_stick_y);
            }
            telemetry.addData("motor power", test.getPower());
            telemetry.addData("button pressed", touchSensor.getState());
            telemetry.update();
        }
    }
}



