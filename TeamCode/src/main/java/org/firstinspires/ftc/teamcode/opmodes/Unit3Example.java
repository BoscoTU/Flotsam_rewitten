package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Unit3Example extends OpMode {
    TankDriveSubsystem tankDriveSubsystem;
    ScanningServoController scanningServoController;
    SpinningServoController spinningServoController;

    Servo scanningServo;
    boolean lastPressedA;
    @Override
    public void init() {
        tankDriveSubsystem = new TankDriveSubsystem(this);

        scanningServo = hardwareMap.get(Servo.class, "scanningServo");
        scanningServoController = new ScanningServoController(this, scanningServo);
        spinningServoController = new SpinningServoController(this);
    }

    @Override
    public void loop() {
        double drive = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        tankDriveSubsystem.drive(drive, turn);
        tankDriveSubsystem.periodic();

        if (!gamepad1.a && lastPressedA) {
            spinningServoController.changeServoState();
            lastPressedA = false;
        } else if (gamepad1.a) {
            lastPressedA = true;
        } else {
            lastPressedA = false;
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        tankDriveSubsystem.drive(0,0);
    }

    public class TankDriveSubsystem {
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

        public void periodic() {
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        }
    }

    public class ScanningServoController {
        static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
        static final int    CYCLE_MS    =   50;     // period of each cycle
        static final double MAX_POS     =  1.0;     // Maximum rotational position
        static final double MIN_POS     =  0.0;     // Minimum rotational position

        // Define class members
        Servo   servo;
        double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
        boolean rampUp = true;
        ElapsedTime timer = new ElapsedTime();

        OpMode opMode;

        public ScanningServoController(OpMode opMode, Servo servo) {
            this.opMode = opMode;
            this.servo = servo;
            timer.reset();
        }

        public void periodic() {
            opMode.telemetry.addData("Servo Position", "%5.2f", position);

            if (timer.milliseconds() >= CYCLE_MS) {
                if (rampUp) {
                    // Keep stepping up until we hit the max value.
                    position += INCREMENT;
                    if (position >= MAX_POS) {
                        position = MAX_POS;
                        rampUp = !rampUp;   // Switch ramp direction
                    }
                    timer.reset();
                } else {
                    // Keep stepping down until we hit the min value.
                    position -= INCREMENT;
                    if (position <= MIN_POS) {
                        position = MIN_POS;
                        rampUp = !rampUp;  // Switch ramp direction
                    }
                    timer.reset();
                }
            }
            // Set the servo to the new position and pause;
            servo.setPosition(position);
        }
    }

    public class SpinningServoController{
        private boolean isSpinning = false;
        private Servo servo;
        private String SERVO_NAME = "spinningServo";
        private double SPINNING_POS = 1.0;
        private double STOPPING_POS = 0.5;

        OpMode opMode;

        public SpinningServoController(OpMode opMode) {
            this.opMode = opMode;
            servo = opMode.hardwareMap.get(Servo.class, SERVO_NAME);
        }

        public void changeServoState() {
            isSpinning = !isSpinning;
        }

        public void periodic() {
            if (isSpinning) {
                servo.setPosition(SPINNING_POS);
            } else {
                servo.setPosition(STOPPING_POS);
            }
        }
    }

}
