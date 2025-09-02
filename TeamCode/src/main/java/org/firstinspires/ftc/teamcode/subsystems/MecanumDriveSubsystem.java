package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDriveSubsystem extends SubsystemBase {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor rearLeftMotor;
    private DcMotor rearRightMotor;

    private IMU imu;
    private OpMode opMode;

    private boolean isFieldCentric;
    public MecanumDriveSubsystem(OpMode opMode) {
        this.opMode = opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        rearRightMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        isFieldCentric = true;
    }

    public void drive(double drive, double strafe, double turn) {
        if (isFieldCentric) {
            double botHeading = getHeading();

            // Rotate the movement direction counter to the bot's rotation
            double rotX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
            double rotY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);

            double frontLeftPower  = Range.clip(rotY + rotX + turn, -1, 1);
            double frontRightPower = Range.clip(rotY - rotX - turn, -1, 1);
            double rearLeftPower   = Range.clip(rotY - rotX + turn, -1, 1);
            double rearRightPower  = Range.clip(rotY + rotX - turn, -1, 1);
            
            frontLeftMotor.setPower(frontLeftPower);
            frontRightMotor.setPower(frontRightPower);
            rearLeftMotor.setPower(rearLeftPower);
            rearRightMotor.setPower(rearRightPower);
        } else {
            double frontLeftPower  = Range.clip(drive + strafe + turn, -1, 1);
            double frontRightPower = Range.clip(drive - strafe - turn, -1, 1);
            double rearLeftPower   = Range.clip(drive - strafe + turn, -1, 1);
            double rearRightPower  = Range.clip(drive + strafe - turn, -1, 1);
            
            frontLeftMotor.setPower(frontLeftPower);
            frontRightMotor.setPower(frontRightPower);
            rearLeftMotor.setPower(rearLeftPower);
            rearRightMotor.setPower(rearRightPower);
        }
    }

    public void resetImu() {
        imu.resetYaw();
    }

    public double getHeading() {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        return botHeading;
    }

    public void switchFieldCentric() {
        isFieldCentric = !isFieldCentric;
    }
    
    @Override
    public void periodic() {
        opMode.telemetry.addData("heading", getHeading());
        opMode.telemetry.addData("is fieldCentric", isFieldCentric);
    }
}
