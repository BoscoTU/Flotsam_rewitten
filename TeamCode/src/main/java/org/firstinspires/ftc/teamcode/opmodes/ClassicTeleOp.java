package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
@Disabled

@TeleOp(name="TeleOp", group="zoidProgram")
public class ClassicTeleOp extends OpMode {
//    TankDriveSubsystem tankDriveSubsystem;
     MecanumDriveSubsystem mecanumDriveSubsystem;
     boolean buttonALastPressed = false;
    @Override
    public void init() {
//        tankDriveSubsystem = new TankDriveSubsystem(this);
        mecanumDriveSubsystem = new MecanumDriveSubsystem(this, true);
    }

    @Override
    public void loop() {
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

//        tankDriveSubsystem.drive(drive, turn);
//        tankDriveSubsystem.periodic();

         mecanumDriveSubsystem.drive(drive, strafe, turn);
         mecanumDriveSubsystem.periodic();
         telemetry.update();

         if (gamepad1.back) {
             mecanumDriveSubsystem.resetImu();
         }
         if (gamepad1.a && !buttonALastPressed) {
             mecanumDriveSubsystem.switchFieldCentric();
             buttonALastPressed = true;
         } else if (gamepad1.a) {
             buttonALastPressed = true;
         } else {
             buttonALastPressed = false;
         }
         if (gamepad1.right_bumper) {
             mecanumDriveSubsystem.changeSpeedMultiplier(0.5);
         } else {
             mecanumDriveSubsystem.changeSpeedMultiplier(1.0);
         }
    }

    @Override
    public void stop() {
//        tankDriveSubsystem.drive(0,0);
         mecanumDriveSubsystem.drive(0,0,0);
    }
}
