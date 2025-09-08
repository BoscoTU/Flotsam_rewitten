package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.roadrunner.MecanumDrive;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDriveSubsystem extends SubsystemBase {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor rearLeftMotor;
    private DcMotor rearRightMotor;

    private IMU imu;
    private OpMode opMode;

    private boolean isFieldCentric;
    private boolean IS_USING_RR = false;
    private double speedMultiplier = 1.0;

    public MecanumDrive mecanumDrive;
    private Pose2d currentPose;

    private final double TOLERANCE = 1.0;
    public MecanumDriveSubsystem(OpMode opMode, boolean isUsingRR) {
        this.opMode = opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;

        IS_USING_RR = isUsingRR;
        if (!IS_USING_RR) {
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

        } else {
            mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        }
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
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

            if (!IS_USING_RR) {
                double frontLeftPower  = Range.clip(rotY + rotX + turn, -1, 1);
                double frontRightPower = Range.clip(rotY - rotX - turn, -1, 1);
                double rearLeftPower   = Range.clip(rotY - rotX + turn, -1, 1);
                double rearRightPower  = Range.clip(rotY + rotX - turn, -1, 1);

                frontLeftMotor.setPower(frontLeftPower);
                frontRightMotor.setPower(frontRightPower);
                rearLeftMotor.setPower(rearLeftPower);
                rearRightMotor.setPower(rearRightPower);
           } else {
               mecanumDrive.setDrivePowers(
                       new PoseVelocity2d(
                               new Vector2d(rotX, rotY), turn
                       ));
           }

        } else {
            if (!IS_USING_RR) {
                double frontLeftPower  = Range.clip(drive + strafe + turn, -1, 1);
                double frontRightPower = Range.clip(drive - strafe - turn, -1, 1);
                double rearLeftPower   = Range.clip(drive - strafe + turn, -1, 1);
                double rearRightPower  = Range.clip(drive + strafe - turn, -1, 1);

                frontLeftMotor.setPower(frontLeftPower);
                frontRightMotor.setPower(frontRightPower);
                rearLeftMotor.setPower(rearLeftPower);
                rearRightMotor.setPower(rearRightPower);
           } else {
               mecanumDrive.setDrivePowers(
                       new PoseVelocity2d(
                               new Vector2d(strafe, drive), turn
                       ));
            }
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

    public void changeSpeedMultiplier(double speedMultiplier) {
        this.speedMultiplier = Range.clip(speedMultiplier, 0, 1);
    }

   public Pose2d getCurrentPos() {
       return mecanumDrive.localizer.getPose();
   }

   @Override
   public void periodic() {
       opMode.telemetry.addData("heading", getHeading());
       opMode.telemetry.addData("is fieldCentric", isFieldCentric);
       if (IS_USING_RR) {
           mecanumDrive.updatePoseEstimate();
           currentPose = mecanumDrive.localizer.getPose();
       }
   }

   public class ToBasket implements Action {
       private boolean cancelled = false;
       @Override
       public boolean run(@NonNull TelemetryPacket telemetryPacket) {
           if((Math.abs(currentPose.position.x) <= TOLERANCE && Math.abs(currentPose.position.y) <= TOLERANCE && Math.abs(currentPose.heading.real) <= TOLERANCE && Math.abs(currentPose.heading.imag) <= TOLERANCE) || cancelled) {
               return false;
           } else {
               return mecanumDrive.actionBuilder(currentPose).
                       strafeToLinearHeading(new Vector2d(0, 0), Math.toRadians(0))
                       .build().run(telemetryPacket);
           }
       }

       public void cancelAbruptly() {cancelled = true;}
   }

   public ToBasket toBasket() {return new ToBasket();}
}