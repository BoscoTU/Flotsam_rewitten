// package org.firstinspires.ftc.teamcode.opmodes;

// import com.qualcomm.robotcore.eventloop.opmode.OpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.Servo;

// @TeleOp(name="Tank Drive TeleOp", group="zoidProgram")
// public class TankDrive extends OpMode {
//     private DcMotor leftMotor;
//     private DcMotor rightMotor;
//     private Servo servo;
//     @Override
//     public void init() {
//         leftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
//         rightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");

//         leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         leftMotor.setDirection(DcMotor.Direction.REVERSE);
//         rightMotor.setDirection(DcMotor.Direction.FORWARD);

// //        servo = hardwareMap.get(Servo.class, "servo");
// //        servo.setPosition(0);
//     }

//     @Override
//     public void loop() {
//         double drive = gamepad1.left_stick_y;
//         double turn = gamepad1.right_stick_x;

//         drive(drive, turn);

// //        if (gamepad1.a) {
// //            servo.setPosition(1);
// //        } else if (gamepad1.b) {
// //            servo.setPosition(0);
// //        }
//         telemetry.addData("servo Pos", servo.getPosition());
//         telemetry.update();
//     }

//     @Override
//     public void stop() {
//         drive(0,0);
//     }

//     public void drive(double drive, double turn) {
//         leftMotor.setPower(rangeCap(drive + turn));
//         rightMotor.setPower(rangeCap(drive - turn));

//         telemetry.addData("Motors", "left (%.2f), right (%.2f)", rangeCap(drive + turn), rangeCap(drive - turn));
//     }

//     public double rangeCap(double x) {
//         if (x > 1.0) {
//             x = 1.0;
//         } else if (x < -1.0) {
//             x = -1.0;
//         }
//         return x;
//     }
// }
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TankDriveSubsystem;
@Disabled

@TeleOp(name="TankDrive TeleOp", group="zoidProgram")
public class TankDrive extends OpMode {
    TankDriveSubsystem tankDriveSubsystem;
    // MecanumDriveSubsystem mecanumDriveSubsystem;
    @Override
    public void init() {
       tankDriveSubsystem = new TankDriveSubsystem(this);
        // mecanumDriveSubsystem = new MecanumDriveSubsystem(this);
    }

    @Override
    public void loop() {
        double drive = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

       tankDriveSubsystem.drive(drive, turn);
       tankDriveSubsystem.periodic();

        //  mecanumDriveSubsystem.drive(drive, strafe, turn);
        //  mecanumDriveSubsystem.periodic();

        //  if (gamepad1.options) {
        //      mecanumDriveSubsystem.resetImu();
        //  }
        //  if (gamepad1.a) {
        //      mecanumDriveSubsystem.switchFieldCentric();
        //  }
    }

    @Override
    public void stop() {
        tankDriveSubsystem.drive(0,0);
        //  mecanumDriveSubsystem.drive(0,0,0);
    }
}