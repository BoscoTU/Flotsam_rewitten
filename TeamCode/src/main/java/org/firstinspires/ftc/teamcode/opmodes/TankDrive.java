package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Tank Drive TeleOp", group="zoidProgram")
public class TankDrive extends OpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private Servo servo;
    @Override
    public void init() {
        leftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

//        servo = hardwareMap.get(Servo.class, "servo");
//        servo.setPosition(0);
    }

    @Override
    public void loop() {
        double drive = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        drive(drive, turn);

//        if (gamepad1.a) {
//            servo.setPosition(1);
//        } else if (gamepad1.b) {
//            servo.setPosition(0);
//        }
        telemetry.addData("servo Pos", servo.getPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        drive(0,0);
    }

    public void drive(double drive, double turn) {
        leftMotor.setPower(rangeCap(drive + turn));
        rightMotor.setPower(rangeCap(drive - turn));

        telemetry.addData("Motors", "left (%.2f), right (%.2f)", rangeCap(drive + turn), rangeCap(drive - turn));
    }

    public double rangeCap(double x) {
        if (x > 1.0) {
            x = 1.0;
        } else if (x < -1.0) {
            x = -1.0;
        }
        return x;
    }
}
