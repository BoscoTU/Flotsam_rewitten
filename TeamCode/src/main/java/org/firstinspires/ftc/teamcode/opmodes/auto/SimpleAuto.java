package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
@Disabled

@Autonomous(name = "SimpleAuto", group = "Scoring Auto")
public class SimpleAuto extends LinearOpMode {

    MecanumDriveSubsystem mecanumDriveSubsystem;

    @Override
    public void runOpMode()  {

        mecanumDriveSubsystem = new MecanumDriveSubsystem(this, true);

        waitForStart();

        TrajectoryActionBuilder outtakePos1 = mecanumDriveSubsystem.mecanumDrive.actionBuilder(new Pose2d( 0,0,0))
                .turn(Math.toRadians(3600));

        Actions.runBlocking(
                outtakePos1.build()
        );
    }

}