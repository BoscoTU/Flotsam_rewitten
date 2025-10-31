package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

@Autonomous(name = "SimpleAuto", group = "Scoring Auto")
public class SimpleAuto extends LinearOpMode {

    MecanumDrive mecanumDrive;
    @Override
    public void runOpMode()  {

        mecanumDrive = new MecanumDrive(this.hardwareMap, new Pose2d(0,0,0));
        waitForStart();

        TrajectoryActionBuilder outtakePos1 = mecanumDrive.actionBuilder(new Pose2d( 0,0,0))
                        .turn(Math.toRadians(3600));

        Actions.runBlocking(
                outtakePos1.build()
        );
    }

}