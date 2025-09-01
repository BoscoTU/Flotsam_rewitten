package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class TankDrive extends OpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    @Override
    public void init() {
        leftMotor = hardwareMap.get(DcMotor.class, "motorLeft");
        rightMotor = hardwareMap.get(DcMotor.class, "motorRight");

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void loop() {
        double drive = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        drive(drive, turn);
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
