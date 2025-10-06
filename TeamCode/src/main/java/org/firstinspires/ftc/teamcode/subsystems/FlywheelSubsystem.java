package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

@Config
public class FlywheelSubsystem extends SubsystemBase {
    private OpMode opMode;

    private DcMotorEx flywheelMotor;
    private final String FLYWHEEL_MOTOR_NAME = "flywheelMotor";
    private final DcMotorSimple.Direction FLYWHEEL_DIRECTION = DcMotorSimple.Direction.FORWARD;

    private final double WHEEL_RADIUS = 4; //inch
    private final double TICKS_PER_ROTATION = 28;
    private double targetVelocity; //ticks per second

    private final PIDFCoefficients FLYWHEEL_PIDF_SETTING = new PIDFCoefficients(200, 0, 0, 0);

    //custom PID + feedforward
    public static class Params {
        public static final double TOLERANCE = 28;
        public static final double kS = -1;
        public static final double kV = -1;
        public static final double kA = -1;

        public static final double P = -1;
        public static final double I = -1;
        public static final double D = -1;
    }

    private PIDController flyWheelController;
    private double maxFlywheelPower = 1.0;


    public FlywheelSubsystem(OpMode opMode) {
        this.opMode = opMode;
        flywheelMotor = opMode.hardwareMap.get(DcMotorEx.class, FLYWHEEL_MOTOR_NAME);
        flywheelMotor.setDirection(FLYWHEEL_DIRECTION);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyWheelController = new PIDController(Params.P, Params.I, Params.D);
    }

    public void runWithDefaultPID(double rpm){
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, FLYWHEEL_PIDF_SETTING);
        targetVelocity = rpmToTps(rpm);
        flywheelMotor.setVelocity(targetVelocity);
    }

    //utilities
    public double getFlywheelPower() {
        return flywheelMotor.getPower();
    }

    public double rpmToTps(double rpm) {
        return rpm * TICKS_PER_ROTATION / 60;
    }

    public double getFlywheelRPM() {
        return flywheelMotor.getVelocity() / TICKS_PER_ROTATION * 60;
    }

    //custom PID
    public void setFlywheelMaxPower(double maxPower) {
        this.maxFlywheelPower = maxPower;
    }

    public void setFlywheelTargetVelocity(double rpm) {
        targetVelocity = rpmToTps(rpm);
    }

    public void setFlywheelMotorPower(double power) {
        if (flywheelMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        flywheelMotor.setPower(power);
    }

    public double flywheelCustomPID(double targetVelocity) {
        if (flywheelMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        double pid = Range.clip(flyWheelController.calculate(flywheelMotor.getVelocity(), targetVelocity), -maxFlywheelPower, maxFlywheelPower);
        return pid;
    }

    public double flywheelFeedForward(double targetVelocity) {
        double targetRadPS = targetVelocity / TICKS_PER_ROTATION * Math.toRadians(360);
        double feedForward = Params.kS * Math.signum(targetRadPS) + Params.kV * targetRadPS;

        return feedForward;
    }

    private void runFlywheelControl() {
        double feedforward = flywheelFeedForward(targetVelocity);
        double pid = flywheelCustomPID(targetVelocity);

        flywheelMotor.setPower(feedforward + pid);
    }

    public class RunFlywheelAction implements Action {

        public RunFlywheelAction(double targetRPM, double maxPower) {
            targetVelocity = rpmToTps(targetRPM);
            maxFlywheelPower = maxPower;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            runFlywheelControl();
            return true;
        }
    }
}
