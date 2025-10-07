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
@Autonomous(name = "circling (constant heading)", group = "zoidProgram")
public class CirclingConstantHeadingAuto extends LinearOpMode {
    private MecanumDrive mecanumDrive;
    @Override
    public void runOpMode()  {
        Pose2d beginPose = new Pose2d( 0, 48, Math.toRadians(90));
        this.mecanumDrive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                mecanumDrive.actionBuilder(beginPose)
                        .stopAndAdd(new SleepAction(5))
                        .strafeToConstantHeading(new Vector2d(0, 58))
                        .setTangent(Math.toRadians(180))

                        .splineToConstantHeading(new Vector2d(-10, 48), Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(0, 38), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(10, 48), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(0, 58), Math.toRadians(180))

                        .splineToConstantHeading(new Vector2d(-10, 48), Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(0, 38), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(10, 48), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(0, 58), Math.toRadians(180))

                        .splineToConstantHeading(new Vector2d(-10, 48), Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(0, 38), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(10, 48), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(0, 58), Math.toRadians(180))

                        .splineToConstantHeading(new Vector2d(-10, 48), Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(0, 38), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(10, 48), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(0, 58), Math.toRadians(180))

                        .splineToConstantHeading(new Vector2d(-10, 48), Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(0, 38), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(10, 48), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(0, 58), Math.toRadians(180))

                        .strafeToSplineHeading(new Vector2d(0, 48), Math.toRadians(90))
                        .build());
    }
}
