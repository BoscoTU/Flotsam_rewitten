package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

import java.util.ArrayList;
import java.util.List;
@TeleOp(name = "RRTeleOp", group = "zoldprograms")
public class RRTeleOp extends OpMode {
   private FtcDashboard dash = FtcDashboard.getInstance();
   private List<Action> runningActions = new ArrayList<>();
   private MecanumDriveSubsystem.ToBasket currentDrivebaseAction = null;

   private MecanumDriveSubsystem driveSubsystem;
   private ElapsedTime timer;
   private boolean pathCalled = false;

   private enum ControlStates {
       DRIVER_CONTROL,
       AUGMENTED_CONTROL
   }
   private ControlStates controlState;
   @Override
   public void init() {
       this.driveSubsystem = new MecanumDriveSubsystem(this, true);
       timer = new ElapsedTime();
       controlState = ControlStates.DRIVER_CONTROL;
   }

   @Override
   public void loop() {
       TelemetryPacket packet = new TelemetryPacket();

       if (gamepad1.a) {
           controlState = ControlStates.AUGMENTED_CONTROL;
       } else {
           controlState = ControlStates.DRIVER_CONTROL;
       }

       if (controlState == ControlStates.DRIVER_CONTROL) {
           if (currentDrivebaseAction != null) {currentDrivebaseAction.cancelAbruptly();}
           runningActions.add(new InstantAction(() -> driveSubsystem.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x)));
       } else if (currentDrivebaseAction == null) {
           currentDrivebaseAction = driveSubsystem.toBasket();
       }
       // update running actions
       List<Action> newActions = new ArrayList<>();
       for (Action action : runningActions) {
           action.preview(packet.fieldOverlay());
           if (action.run(packet)) {
               newActions.add(action);
           }
       }
       if (!(currentDrivebaseAction == null)) {
           currentDrivebaseAction.preview(packet.fieldOverlay());
           if (!currentDrivebaseAction.run(packet)) {
               currentDrivebaseAction = null;
           }
       }
       runningActions = newActions;
       driveSubsystem.periodic();
       dash.sendTelemetryPacket(packet);
       Pose2d currentPos = driveSubsystem.getCurrentPos();
       telemetry.addData("x", currentPos.position.x);
       telemetry.addData("y", currentPos.position.y);
       telemetry.addData("heading real", currentPos.heading.real);
       telemetry.addData("heading imag", currentPos.heading.imag);
       telemetry.addData("running actions", currentDrivebaseAction);
       telemetry.addData("timer", timer.seconds());
   }
}