package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TankDriveSubsystem;

@TeleOp(name="TeleOp", group="zoidProgram")
public class ClassicTeleOp extends OpMode {
//    TankDriveSubsystem tankDriveSubsystem;
     MecanumDriveSubsystem mecanumDriveSubsystem;
    @Override
    public void init() {
//        tankDriveSubsystem = new TankDriveSubsystem(this);
        mecanumDriveSubsystem = new MecanumDriveSubsystem(this);
    }

    @Override
    public void loop() {
        double drive = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

//        tankDriveSubsystem.drive(drive, turn);
//        tankDriveSubsystem.periodic();

         mecanumDriveSubsystem.drive(drive, strafe, turn);
         mecanumDriveSubsystem.periodic();

         if (gamepad1.options) {
             mecanumDriveSubsystem.resetImu();
         }
         if (gamepad1.a) {
             mecanumDriveSubsystem.switchFieldCentric();
         }
    }

    @Override
    public void stop() {
//        tankDriveSubsystem.drive(0,0);
         mecanumDriveSubsystem.drive(0,0,0);
    }
}
