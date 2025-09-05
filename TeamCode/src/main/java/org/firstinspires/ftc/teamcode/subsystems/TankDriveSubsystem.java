package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class TankDriveSubsystem extends SubsystemBase {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor rearLeftMotor;
    private DcMotor rearRightMotor;

    private OpMode opMode;

    private double leftPower;
    private double rightPower;

    public TankDriveSubsystem(OpMode opMode) {
        this.opMode = opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "rearLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        rearRightMotor = hardwareMap.get(DcMotor.class, "rearRightMotor");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void drive(double drive, double turn) {
        leftPower = Range.clip(drive + turn, -1, 1);
        rightPower = Range.clip(drive - turn, -1, 1);

        frontLeftMotor.setPower(leftPower);
        rearLeftMotor.setPower(leftPower);
        frontRightMotor.setPower(rightPower);
        rearRightMotor.setPower(rightPower);
    }

    @Override
    public void periodic() {
        opMode.telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }
}
