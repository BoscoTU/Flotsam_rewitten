package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
@TeleOp(name = "8shape Auto", group = "zoidProgram")
public class TwoDBackAndForthAuto extends LinearOpMode {
    private MecanumDrive mecanumDrive;
    private LimelightSubsystem limelightSubsystem;
    @Override
    public void runOpMode()  {
        Pose2d beginPose = new Pose2d(0,48 , Math.toRadians(0));
        this.mecanumDrive = new MecanumDrive(hardwareMap, beginPose);
//        this.limelightSubsystem = new LimelightSubsystem(this);

        waitForStart();

        for (int i = 1; i <= 10; i++) {
            Actions.runBlocking(
                    mecanumDrive.actionBuilder(new Pose2d(0, 35, Math.toRadians(90)))
                    .setTangent(Math.toRadians(270))
                    .splineToSplineHeading(new Pose2d(30, 20, Math.toRadians(90)), Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(60, 35, Math.toRadians(180)), Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(30, 55, Math.toRadians(270)), Math.toRadians(180))
                    .splineToSplineHeading(new Pose2d(0, 35, Math.toRadians(90)), Math.toRadians(270))

                    .splineToSplineHeading(new Pose2d(-30, 20, Math.toRadians(180)), Math.toRadians(180))
                    .splineToSplineHeading(new Pose2d(-60, 35, Math.toRadians(90)), Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(-30, 55, Math.toRadians(180)), Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(0, 35, Math.toRadians(90)), Math.toRadians(270))
                    .build());

                    while(!gamepad1.a) {
                        mecanumDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0));
//                        getRobotPose();
                    }
        }

    }


    private void getRobotPose() {
        mecanumDrive.updatePoseEstimate();
        double[] limelightPose = limelightSubsystem.getRobotPoseOnField();
        telemetry.addData("RR cood: ", "X: %.3f, Y: %.3f, Heading: %.3f", mecanumDrive.localizer.getPose().position.x, mecanumDrive.localizer.getPose().position.y, Math.toDegrees(mecanumDrive.localizer.getPose().heading.toDouble()));
        telemetry.addData("LL cood: ", "X: %.3f, Y: %.3f, Heading: %.3f", limelightPose[0] * limelightSubsystem.METER_TO_INCH, limelightPose[1] * limelightSubsystem.METER_TO_INCH, limelightPose[2]);
        telemetry.update();
    }
}
