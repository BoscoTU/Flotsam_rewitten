package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ArmSubsystem extends SubsystemBase {
    // Constants
    public static final String ELBOW_MOTOR_NAME = "sampElbow";
    public static final DcMotor.Direction ELBOW_DIRECTION = DcMotor.Direction.FORWARD;
    public static double ELBOW_P = 0.013;
    public static double ELBOW_I = 0;
    public static double ELBOW_D = 0.0001;

    public static final int ELBOW_MAX_POSITION = 685;
    public static final int ELBOW_MIN_POSITION = 0;
    public static final int TICKS_PER_ELBOW_ROTATION = 2740;
    public static final double POWER_TO_HOLD_ARM = 0.263;
    public static final int TOLERANCE = 25;

    // Hardware Components
    private final DcMotorEx elbowMotor;

    // PID Controller
    private final PIDController elbowController;

    private int targetElbowPosition;
    private double maxElbowPower;
    private final Telemetry telemetry;
    private final ElapsedTime elbowTimer;
    private boolean updateFirstCall;
    private boolean elbowPIDtimeout = false;

    private int previousElbowTarget = 0;
    private int previousElbowPosition = 0;

    public enum ArmPosition {
        INTAKE_POSITION(0),
        HIGH_OUTTAKE_POSITION(685),
        LOW_OUTTAKE_POSITION(685);

        public final int elbowPos;

        ArmPosition(int elbowPos) {
            this.elbowPos = elbowPos;
        }
    }

    public ArmSubsystem(OpMode opMode) {
        telemetry = opMode.telemetry;

        elbowMotor = opMode.hardwareMap.get(DcMotorEx.class, ELBOW_MOTOR_NAME);

        elbowMotor.setDirection(ELBOW_DIRECTION);
        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setTargetElbowPosition(ELBOW_MIN_POSITION);
        setElbowMaxPower(1.0);

        elbowController = new PIDController(ELBOW_P, ELBOW_I, ELBOW_D);

        updateFirstCall = true;
        elbowTimer = new ElapsedTime();
        elbowTimer.reset();
    }

    public void setElbowMaxPower(double power) {
        maxElbowPower = Range.clip(power, -1.0, 1.0);
    }

    public boolean getElbowAtTarget() {
        return Math.abs(elbowMotor.getCurrentPosition() - targetElbowPosition) <= TOLERANCE;
    }

    public void setTargetElbowPosition(int target) {
        targetElbowPosition = Range.clip(target, ELBOW_MIN_POSITION, ELBOW_MAX_POSITION);
    }

    public double getElbowDegrees() {
        return (double) elbowMotor.getCurrentPosition() / TICKS_PER_ELBOW_ROTATION * 360.0;
    }

    public void setTargetArmPosition(ArmPosition armPosition) {
        setTargetElbowPosition(armPosition.elbowPos);
    }

    public void resetEncoders() {
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public SequentialCommandGroup getMoveArmToPositionCommand(ArmPosition position, double maxElbowPowerGoingUp, double maxElbowPowerGoingDown) {
        double previousElbowMaxPower = this.maxElbowPower;

        return new SequentialCommandGroup(
                new ElbowToPositionCommand(this, position.elbowPos,
                        (position.elbowPos < elbowMotor.getCurrentPosition()) ? maxElbowPowerGoingDown : maxElbowPowerGoingUp),
                new RunCommand(() -> setElbowMaxPower(previousElbowMaxPower))
        );
    }

    public SequentialCommandGroup getMoveArmToPositionCommand(ArmPosition position) {
        return new SequentialCommandGroup(
                new ElbowToPositionCommand(this, position.elbowPos)
        );
    }

    public void addToElbowTarget(int input) {
        setTargetElbowPosition(targetElbowPosition + input);
    }

    public void addToElbowTarget(int input, boolean overrideLimits) {
        if (overrideLimits) {
            targetElbowPosition += input;
        } else {
            setTargetElbowPosition(targetElbowPosition + input);
        }
    }

    private void runElbowPID() {
        double pid = Range.clip(elbowController.calculate(elbowMotor.getCurrentPosition(), targetElbowPosition), -maxElbowPower, maxElbowPower);
        double feedForward = Math.cos(Math.toRadians(getElbowDegrees())) * POWER_TO_HOLD_ARM;

        if (Math.abs(elbowMotor.getCurrentPosition() - ELBOW_MIN_POSITION) <= TOLERANCE) {
            feedForward = 0;
        }

        telemetry.addData("Elbow Feed Forward", feedForward);

        if (Math.abs(elbowMotor.getCurrentPosition() - previousElbowPosition) > 2) {
            elbowTimer.reset();
            previousElbowTarget = targetElbowPosition;
            previousElbowPosition = elbowMotor.getCurrentPosition();
            elbowMotor.setPower(pid + feedForward);
            elbowPIDtimeout = false;
        } else if (elbowTimer.seconds() > 1 && previousElbowTarget == targetElbowPosition) {
            elbowMotor.setPower(feedForward);
            elbowPIDtimeout = true;
        } else {
            elbowMotor.setPower(pid + feedForward);
            elbowPIDtimeout = false;
        }
    }

    public class ElbowToPositionAction implements Action {
        private final int target;
        private final double maxPower;
        private double previousMaxPower;

        public ElbowToPositionAction(ArmSubsystem armSubsystem, int target, double maxPower) {
            this.target = target;
            this.maxPower = maxPower;
        }

        public ElbowToPositionAction(ArmSubsystem armSubsystem, int target) {
            this.target = target;
            this.maxPower = armSubsystem.maxElbowPower;
        }


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

        }
    }

    public static class ElbowToPositionCommand extends CommandBase {
        private final ArmSubsystem armSubsystem;
        private final int target;
        private final double maxPower;
        private double previousMaxPower;

        public ElbowToPositionCommand(ArmSubsystem armSubsystem, int target) {
            this.armSubsystem = armSubsystem;
            this.target = target;
            this.maxPower = armSubsystem.maxElbowPower;
            addRequirements(armSubsystem);
        }

        public ElbowToPositionCommand(ArmSubsystem armSubsystem, int target, double maxPower) {
            this.armSubsystem = armSubsystem;
            this.target = target;
            this.maxPower = maxPower;
            addRequirements(armSubsystem);
        }

        @Override
        public void initialize() {
            previousMaxPower = armSubsystem.maxElbowPower;
            armSubsystem.setElbowMaxPower(maxPower);
            armSubsystem.setTargetElbowPosition(target);
            armSubsystem.elbowTimer.reset();
            armSubsystem.elbowPIDtimeout = false;
        }

        @Override
        public boolean isFinished() {
            return armSubsystem.getElbowAtTarget() || armSubsystem.elbowPIDtimeout;
        }

        @Override
        public void end(boolean interrupted) {
            armSubsystem.setElbowMaxPower(previousMaxPower);
        }
    }

    @Override
    public void periodic() {
        if (updateFirstCall) {
            elbowTimer.reset();
            updateFirstCall = false;
        }

        telemetry.addData("Elbow Position", elbowMotor.getCurrentPosition());
        telemetry.addData("Arm Target", targetElbowPosition);
        telemetry.addData("Elbow Degrees", getElbowDegrees());

        runElbowPID();
    }
}
