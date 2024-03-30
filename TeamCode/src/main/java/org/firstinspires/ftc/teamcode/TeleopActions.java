package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helpers.PositionsClass;
import org.firstinspires.ftc.teamcode.helpers.PositionsClass.IntakePosition;
import org.firstinspires.ftc.teamcode.helpers.ActionOpMode;
import org.firstinspires.ftc.teamcode.helpers.PIDFController;
import org.firstinspires.ftc.teamcode.helpers.PoseStorage;
import org.firstinspires.ftc.teamcode.helpers.hardware.MotorActions;
import org.firstinspires.ftc.teamcode.helpers.hardware.MotorControl;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Teleop Field Centric")
@Config
public class TeleopActions extends ActionOpMode {


    // Declare a PIDF Controller to regulate heading
    private final PIDFController.PIDCoefficients HEADING_PID_JOYSTICK = new PIDFController.PIDCoefficients(0.2, 0.0, 1);
    private final PIDFController joystickHeadingController = new PIDFController(HEADING_PID_JOYSTICK);
    private final PIDFController.PIDCoefficients PIXEL_PID_JOYSTICK = new PIDFController.PIDCoefficients(0.004, 0.0, 0.0);
    private final PIDFController pixelHeadingController = new PIDFController(PIXEL_PID_JOYSTICK);
    double speed;
    LynxModule CONTROL_HUB;
    LynxModule EXPANSION_HUB;
    boolean fieldCentric = true;
    public MecanumDrive drive;
    public Servo plane;
    List<Action> runningActions = new ArrayList<>();
    final ElapsedTime liftTimer = new ElapsedTime();
    final ElapsedTime loopTime = new ElapsedTime();
    boolean pixelInClaw = false;
    boolean pixelInHook = true;//false;
    final Gamepad currentGamepad1 = new Gamepad();
    final Gamepad currentGamepad2 = new Gamepad();
    final Gamepad previousGamepad1 = new Gamepad();
    final Gamepad previousGamepad2 = new Gamepad();

    boolean showMotorTelemetry = true;
    boolean showStateTelemetry = true;
    boolean showLoopTimes = true;
    boolean showTelemetryMenu = false;
    boolean showPoseTelemetry = true;
    boolean showCameraTelemetry = false;

    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BR;
    private DcMotor BL;
    MotorControl motorControl;
    MotorActions motorActions;
    boolean drivingEnabled = true;
    boolean actionRunning = false;
    boolean suspendSet = false;


    @Override
    public void runOpMode() {
        //PhotonCore.enable();

        //  Initialization Period
        FL = hardwareMap.dcMotor.get("frontLeft");
        FR = hardwareMap.dcMotor.get("frontRight");
        BL = hardwareMap.dcMotor.get("rearLeft");
        BR = hardwareMap.dcMotor.get("rearRight");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        // Enable Bulk Caching
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        CONTROL_HUB = allHubs.get(0);
        EXPANSION_HUB = allHubs.get(1);

        // RoadRunner Init


        // Telemetry Init
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Motor Init
        motorControl = new MotorControl(hardwareMap);
        motorActions = new MotorActions(motorControl);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        if (isStopRequested()) return;


        // Run Period

        while (opModeIsActive() && !isStopRequested()) {
            IntakePosition current = motorActions.currentIn;
            PositionsClass.OuttakeAngle currentAngle = motorActions.currentAngle;
            PositionsClass.OuttakePosition currentPos = motorActions.currentOut;
            PositionsClass.OuttakePositionRight currentRight = motorActions.currentRight;
            // Reset measured loop time
            loopTime.reset();
            // Reset bulk cache
            allHubs.forEach(LynxModule::clearBulkCache);

            // This lets us do reliable rising edge detection, even if it changes mid loop
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            float rightStickY = gamepad2.right_stick_y;

            // Manual Control
            double padSlideControl = gamepad2.left_stick_y;
            double padSlideControlMultiplier = 40;
            // Misc
            boolean padForceDown = gamepad2.dpad_down && gamepad2.options;


            double r = Math.hypot(-gamepad1.right_stick_x, gamepad1.right_stick_y);
            double robotAngle = Math.atan2(gamepad1.right_stick_y, -gamepad1.right_stick_x) - Math.PI / 4;
            double rightX = gamepad1.left_stick_x / 1.5;
            final double v1 = -r * Math.cos(robotAngle) + rightX;
            final double v2 = -r * Math.sin(robotAngle) - rightX;
            final double v3 = -r * Math.sin(robotAngle) + rightX;
            final double v4 = -r * Math.cos(robotAngle) - rightX;

            // Set motor powers
            FL.setPower(v1 * 1);
            FR.setPower(v2 * 1);
            BL.setPower(v3 * 1);
            BR.setPower(v4 * 1);



            // Slide (Manual)

            if (!actionRunning){
                double targ = (motorControl.slide.getTargetPosition() + (padSlideControl * padSlideControlMultiplier));
                motorControl.slide.setTargetPosition(Math.min(targ, 0));

            }

            if (gamepad2.dpad_left && !previousGamepad2.dpad_left && !current.equals(IntakePosition.Transfer)) {
                run(transfer());
            }

            if (gamepad2.dpad_right&& !previousGamepad2.dpad_right && !current.equals(IntakePosition.Intake)){
                run(intake());
            }

            if (motorControl.slide.getTargetPosition() >  300) {
                run(motorActions.ClawOpen());
            }
            else {
                run(motorActions.ClawClose());
            }

            if (gamepad2.a && !previousGamepad2.a){
                if (currentAngle.equals(PositionsClass.OuttakeAngle.Angled)){
                    run(motorActions.flipperDown());
                }
                else {
                    run(motorActions.flipperUp());
                }
            }
            if (gamepad2.left_bumper && !previousGamepad2.left_bumper){
                if (currentPos.equals(PositionsClass.OuttakePosition.Open)){
                    run(motorActions.OutTakeLeft.Close());
                }
                else {
                    run(motorActions.OutTakeLeft.Open());
                }
            }

            if (gamepad2.right_bumper && !previousGamepad2.right_bumper){
                if (currentRight.equals(PositionsClass.OuttakePositionRight.Open)){
                    run(motorActions.OutTakeRight.Close());
                }
                else {
                    run(motorActions.OutTakeRight.Open());
                }
            }

            if (gamepad2.left_trigger > 0) {
                run(motorActions.flopper.intakePixel());
            }
            else {
                run(motorActions.flopper.stop());
                run(motorActions.flopper.reset());
            }

            /*
            Action currentServoAction =  motorActions.IntakeArmServo.Fifth();
            if (rightStickY == 0.0 ) {
                run(currentServoAction);
            }
            else if (rightStickY < -0.8) {
                currentServoAction = motorActions.IntakeArmServo.Fifth();
                run(moveBox(motorActions.IntakeArmServo.Fifth()));
            } else if (rightStickY >= -0.8 && rightStickY < -0.4) {
                currentServoAction = motorActions.IntakeArmServo.Fourth();
                run(moveBox(motorActions.IntakeArmServo.Fourth()));
            } else if (rightStickY >= -0.4 && rightStickY < 0.4) {
                currentServoAction = motorActions.IntakeArmServo.Third();
                run(moveBox(motorActions.IntakeArmServo.Third()));
            } else if (rightStickY >= 0.4 && rightStickY < 0.8) {
                currentServoAction = motorActions.IntakeArmServo.Second();
                run(moveBox(motorActions.IntakeArmServo.Second()));
            } else if (rightStickY >= 0.8) {
                currentServoAction = motorActions.IntakeArmServo.Last();
                run(moveBox(motorActions.IntakeArmServo.Last()));
            }*/


            TelemetryPacket packet = new TelemetryPacket();
            updateAsync(packet);
            motorControl.update();
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

        }
    }


    // TODO: probably not needed, just make a normal action
    interface input {
        boolean isPressed();
    }

    Action intake (){
        return new SequentialAction(
                new InstantAction(() -> actionRunning = true),
                motorActions.intake(),
                new InstantAction(() -> actionRunning = false)

        );

    }

    Action moveBox (Action action){
        return new SequentialAction(
                action,
                new SleepAction(0.5)

        );
    }
    Action transfer(){
        return new SequentialAction(
                new InstantAction(() -> actionRunning = true),
                motorActions.transfer(),
                new InstantAction(() -> actionRunning = false)

        );
    }
    Action waitForInput (input input){
        return telemetryPacket -> input.isPressed();
    }
}
