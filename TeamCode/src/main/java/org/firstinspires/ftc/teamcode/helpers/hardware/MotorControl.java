package org.firstinspires.ftc.teamcode.helpers.hardware;


import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This class is used to control the motor systems on the robot.
 */
public class MotorControl {
    private static PIDController liftController;
    private static PIDController IntakeArmController;
    private static PIDController SpinnerController;


    public final CachingServo intakeArmLeft;
    public final CachingServo intakeArmRight;
    public final CachingServo outTakeClawLeft;
    public final CachingServo outTakeClawRight;
    public final CachingServo outTakeArm;
    public final CachingServo armGate;

    public final Slide slide;
    public final IntakeArm intakeArm;
    public final Flopper flopper;


    /**
     * This initializes the arm and slide motors, and resets the mode to the default. This should be run before any other methods.
     *
     * @param hardwareMap The hardware map to use to get the motors.
     */
    public MotorControl(@NonNull HardwareMap hardwareMap) {
        // TODO; probably not needed to automate init, but if you did use annotations
        slide = new Slide(hardwareMap);
        intakeArm = new IntakeArm(hardwareMap);
        flopper = new Flopper(hardwareMap);
        intakeArmLeft = new CachingServo(hardwareMap.get(Servo.class, "boxLeft"));
        intakeArmRight =  new CachingServo(hardwareMap.get(Servo.class, "boxRight"), 0.0001);
        outTakeClawLeft =  new CachingServo(hardwareMap.get(Servo.class, "clawLeft"));
        outTakeClawRight =  new CachingServo(hardwareMap.get(Servo.class, "clawRight"));
        outTakeArm =  new CachingServo(hardwareMap.get(Servo.class, "flipper"));
        armGate = new CachingServo(hardwareMap.get(Servo.class, "armGate"));
        armGate.setPosition(0);
    }

    /**
     * This class updates the arm and slide motors to match the current state.
     */
    public void update() {
        slide.update();
        flopper.update();
        float currentArmPos = intakeArm.motor.getCurrentPosition();
        double currentArmTarget = intakeArm.targetPosition;
    }




    public static class IntakeArm extends ControlledMotor {
        private static double p =0.01, i = 0, d = 0.0003;
        public static double f = 0.1;
        private final double ticks_in_degree = 1993.6/180;
        double thresh = 0.6;
        boolean resetting = false;



        /**
         * This initializes the slide motor. This should be run before any other methods.
         *
         * @param hardwareMap The hardware map to use to get the motors.
         */
        public IntakeArm(HardwareMap hardwareMap) {
            IntakeArmController = new PIDController(p,i,d);
            motor = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, "boxLifter"), 0.005);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        }

        /**
         * This stops the slide, sets the state to down, sets the target to 0, and resets the encoder.
         */
        public void reset() {
            motor.setPower(0);
            targetPosition = 0;
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        }

        @Override
        public void setTargetPosition(double targetPosition) {
            this.targetPosition = targetPosition;
            motor.setTargetPosition((int) targetPosition);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public void setPower(float power){
            motor.setPower(power);
        }
        /**
         * This updates the slide motor to match the current state. This should be run in a loop.
         */
        public void update() {
        }

        public void findZero() {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(-0.5);
            resetting = true;
        }


        /**
         * Checks if the motor is close enough to the target position.
         *
         * @return boolean indicating whether the current position is close to the target.
         */
        public boolean closeEnough() {
            return Math.abs(motor.getCurrentPosition() - targetPosition) < 20;  
        }


    }

    public static class Flopper extends ControlledMotor {
        private static double p =0.007, i = 0, d = 0;

        boolean resetting = false;

        /**
         * This initializes the slide motor. This should be run before any other methods.
         *
         * @param hardwareMap The hardware map to use to get the motors.
         */
        public Flopper(HardwareMap hardwareMap) {
            SpinnerController = new PIDController(p,i,d);
            motor = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, "spinner"), 0.05);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        }

        /**
         * This stops the slide, sets the state to down, sets the target to 0, and resets the encoder.
         */
        public void reset() {
            motor.setPower(0);
            targetPosition = 0;
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        }


        /**
         * This updates the slide motor to match the current state. This should be run in a loop.
         */
        public void update() {
            SpinnerController.setPID(p,i,d);
            int pos = motor.getCurrentPosition();
            double pid = SpinnerController.calculate(pos, targetPosition);

            double power = pid;

            if (motor.isOverCurrent()) {
                reset();
            } else {
                motor.setPower(power);
            }
        }


        public void findZero() {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPower(-0.5);
            resetting = true;
        }

        @Override
        public void setTargetPosition(double targetPosition) {
            this.targetPosition = targetPosition + motor.getCurrentPosition();
        }
        /**
         * Checks if the motor is close enough to the target position.
         *
         * @return boolean indicating whether the current position is close to the target.
         */
        public boolean closeEnough() {
            // You might want to check both motors if they need to be in sync
            return Math.abs(motor.getCurrentPosition() - targetPosition) < 20;
        }


    }

    /**
     * This class controls the slide motor.
     */
    public static class Slide extends ControlledMotor {

        boolean resetting = false;
        DcMotorEx motor2;
        private static double p = 0.0045, i = 0, d = 0.0003;
        public static double f = 0.001;
        private final double ticks_in_degrees = 751.8/180;
        /**
         * This initializes the slide motor. This should be run before any other methods.
         *
         * @param hardwareMap The hardware map to use to get the motors.
         */
        public Slide(HardwareMap hardwareMap) {
            liftController = new PIDController(p,i,d);
            liftController.setPID(p,i,d);
            motor = hardwareMap.get(DcMotorEx.class, "liftLeft");
            motor2 = hardwareMap.get(DcMotorEx.class, "liftRight");
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor2.setDirection(DcMotor.Direction.REVERSE);
        }

        /**
         * This stops the slide, sets the state to down, sets the target to 0, and resets the encoder.
         */
        public void reset() {
            motor.setPower(0);
            motor2.setPower(0);
            targetPosition = 0;
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        /**
         * This updates the slide motor to match the current state. This should be run in a loop.
         */
        public void update() {
            liftController.setPID(p,i,d);
            int pos = motor.getCurrentPosition();
            double pid = liftController.calculate(pos, targetPosition);
            double ff = Math.cos(Math.toRadians(targetPosition/ ticks_in_degrees)) * f;


            double power = pid + ff;

            motor.setPower(power);
            motor2.setPower(power);
        }



       /**
        * Checks if the motor is close enough to the target position.
        *
        * @return boolean indicating whether the current position is close to the target.
        */
        public boolean closeEnough() {
            // You might want to check both motors if they need to be in sync
            return Math.abs(motor.getCurrentPosition() - targetPosition) < 40 ;
        }


    }

    /**
     * This class controls the claw.
     */

    public abstract static class ControlledMotor {
        public DcMotorEx motor;
        double targetPosition;
        public double getTargetPosition() {
            return targetPosition;
        }
        public void setTargetPosition(double targetPosition) {
            this.targetPosition = targetPosition;
        }


        public abstract void update();
        public abstract void reset();
        public abstract boolean closeEnough();
        public boolean isOverCurrent() {
            return motor.isOverCurrent();
        }
    }
}