package org.firstinspires.ftc.teamcode.helpers.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.helpers.PositionsClass;

public class MotorActions {
    public final MotorControl motorControl;
    public PositionsClass.IntakePosition currentIn = PositionsClass.IntakePosition.Start;
    public PositionsClass.OuttakePosition currentOut = PositionsClass.OuttakePosition.Open;
    public PositionsClass.OuttakeAngle currentAngle = PositionsClass.OuttakeAngle.Down;
    public PositionsClass.OuttakePositionRight currentRight = PositionsClass.OuttakePositionRight.Open;

    // Motor
    public final Slide slide;
    public final IntakeArm intakeArm;
    public final Flopper flopper;

    // Servo
    public final IntakeArmServo IntakeArmServo;
    public final OutTakeLeft OutTakeLeft;
    public final OutTakeRight OutTakeRight;
    public final OutTakeArm OutTakeArm;
    public final ArmGate armGate;



    public MotorActions(MotorControl motorControl) {
        this.motorControl = motorControl;
        this.slide = new Slide();
        this.intakeArm = new IntakeArm();
        this.flopper = new Flopper();
        this.IntakeArmServo = new IntakeArmServo();
        this.OutTakeLeft = new OutTakeLeft();
        this.OutTakeRight = new OutTakeRight();
        this.OutTakeArm = new OutTakeArm();
        this.armGate = new ArmGate();
    }
    public Action ClawClose() {

        return new ParallelAction(
                OutTakeLeft.Close(),
                OutTakeRight.Close()
        );
    }

    public Action ClawOpen() {

        return new ParallelAction(
                OutTakeLeft.Open(),
                OutTakeRight.Open()
        );
    }
    public Action intake(){
        currentIn = PositionsClass.IntakePosition.Intake;
        return new SequentialAction(
                armGate.Close(),
                IntakeArmServo.Zero(),
                intakeArm.Intake(),
                intakeArm.waitUntilFinished()
        );
    }

    public Action placePixelOnMarker(){
        return  new SequentialAction(
                intakeArm.midFromLow(),
                IntakeArmServo.PixelOut(),
                intakeArm.waitUntilFinished(),
                flopper.depositPixel()
        );
    }

    public Action transfer(){
        currentIn = PositionsClass.IntakePosition.Transfer;
        return new SequentialAction(
                ClawOpen(),
                flopper.hold(),
                IntakeArmServo.Transfer(),
                intakeArm.midFromLow(),
                intakeArm.waitUntilFinished(),
                IntakeArmServo.Idle(),
                intakeArm.Transfer(),
                intakeArm.waitUntilFinished(),
                armGate.Open(),
                new SleepAction(0.5),
                flopper.stop(),
                        new SleepAction(0.5),
                ClawClose()

        );
    }

    public Action pixelBackDrop(double pos){
        return new SequentialAction(
                ClawClose(),
                slide.Clearence(),
                slide.waitUntilFinished(),
                slide.setTargetPosition(pos),
                OutTakeArm.Up()
        );
    }
    public Action liftDown(){
        return new SequentialAction(
                OutTakeArm.Down(),
                slide.downPosition()
        );
    }
    public Action flipperDown(){
        currentAngle = PositionsClass.OuttakeAngle.Down;
        return OutTakeArm.Down();
    }
    public Action flipperUp(){
        currentAngle = PositionsClass.OuttakeAngle.Angled ;
        return OutTakeArm.Up();
    }


    public Action update() {
        return t -> {
            motorControl.update();
            return true; // this returns true to make it loop forever; use RaceParallelCommand
        };
    }



 public class Slide {
        public Action setTargetPosition(double position) {
            return t -> {
                motorControl.slide.setTargetPosition(position);
                return false;
            };
        }
        public Action waitUntilFinished() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    return !motorControl.slide.closeEnough();
                }
            };
        }

     public Action reset() {
         return new SequentialAction(t -> {motorControl.slide.reset();return false;},
                 new SleepAction(0.4));
     }



     public Action downPosition() {
         return setTargetPosition(0);
     }
     public Action Clearence() {
         return setTargetPosition(100);
     }
     public Action first() {
         return setTargetPosition(400);
     }
     public Action second() {
         return setTargetPosition(700);
     }
    }
    public class IntakeArm {
        public Action setTargetPosition(double position, float power) {
            return t -> {
                motorControl.intakeArm.setTargetPosition(position);
                motorControl.intakeArm.setPower(power);
                return false;
            };
        }
        public Action waitUntilFinished() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    return !motorControl.intakeArm.closeEnough();
                }
            };
        }
        public Action reset() {
            return new SequentialAction(t -> {motorControl.intakeArm.reset();return false;},
                    new SleepAction(0.1));
        }

        public Action Transfer() {
            return setTargetPosition(635, 0.3f);
        }
        public Action midFromLow() {
            return setTargetPosition(250, 0.3f);
        }
        public Action midFromHigh() {
            return setTargetPosition(250, 0.3f);
        }
        public Action Intake() {
            return setTargetPosition(-10, 0.3f);
        }

    }

    public class Flopper {
        public Action setTargetPosition(double position) {
            return t -> {
                motorControl.flopper.setTargetPosition(position);
                return false;
            };
        }
        public Action waitUntilFinished() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    return !motorControl.flopper.closeEnough();
                }
            };
        }

        public Action reset() {
            return new SequentialAction(t -> {motorControl.flopper.reset();return false;});
        }

        public Action stop() {
            return new SequentialAction(t -> {
                motorControl.flopper.setTargetPosition(motorControl.flopper.getTargetPosition());
                ;return false;});
        }
        public Action depositPixel() {
            return setTargetPosition(150);
        }
        public Action intakePixel() {

            return setTargetPosition(-3000);
        }
        public Action hold() {

            return setTargetPosition(-20000);
        }
    }
    public class IntakeArmServo {
        public Action Fifth() {
            return new SequentialAction(t -> {
                motorControl.intakeArmLeft.setPosition(0.45);
                return false;
            },
                    new SleepAction(0.3));
        }
        public Action Fourth() {
            return new SequentialAction(t -> {
                motorControl.intakeArmLeft.setPosition(0.50);
                return false;
            },
                    new SleepAction(0.3));
        }
        public Action Third() {
            return new SequentialAction(t -> {
                motorControl.intakeArmLeft.setPosition(0.505);
                return false;
            },
                    new SleepAction(0.3));
        }
        public Action Second() {
            return new SequentialAction(t -> {
                motorControl.intakeArmLeft.setPosition(0.510);
                return false;
            },
                    new SleepAction(0.3));
        }
        public Action Last() {
            return new SequentialAction(t -> {
                motorControl.intakeArmLeft.setPosition(0.515);
                return false;
            },
                    new SleepAction(0.3));
        }
        public Action Transfer() {
            return new SequentialAction(t -> {
                motorControl.intakeArmLeft.setPosition(0.7);
                return false;
            },
                    new SleepAction(0.3));
        }
        public Action PixelOut() {
            return new SequentialAction(t -> {
                motorControl.intakeArmLeft.setPosition(1);
                return false;
            },
                    new SleepAction(0.3));
        }
        public Action Zero() {
            return new SequentialAction(t -> {
                motorControl.intakeArmLeft.setPosition(0);

                return false;
            },
                    new SleepAction(0.3));
        }
        public Action Idle() {
            return new SequentialAction(t -> {
                motorControl.intakeArmLeft.setPosition(1);

                return false;
            },
                    new SleepAction(0.3));
        }

    }

    public class OutTakeLeft {
        public Action Close() {
            currentOut = PositionsClass.OuttakePosition.Close;
            return new SequentialAction(t -> {
                motorControl.outTakeClawLeft.setPosition(0.5);
                return false;
            },
                    new SleepAction(0.3));
        }
        public Action Open() {
            currentOut = PositionsClass.OuttakePosition.Open;
            return new SequentialAction(t -> {
                motorControl.outTakeClawLeft.setPosition(0);
                return false;
            },
                    new SleepAction(0.3));
        }
    }

    public class ArmGate {
        public Action Open() {
            return new SequentialAction(t -> {
                motorControl.armGate.setPosition(0.5);
                return false;
            },
                    new SleepAction(0.3));
        }
        public Action Close() {
            return new SequentialAction(t -> {
                motorControl.armGate.setPosition(0);
                return false;
            },
                    new SleepAction(0.3));
        }
    }

    public class OutTakeRight {
        public Action Close() {
            currentRight = PositionsClass.OuttakePositionRight.Close;
            return new SequentialAction(t -> {
                motorControl.outTakeClawRight.setPosition(0.4);
                return false;
            },
                    new SleepAction(0.3));
        }
        public Action Open() {
            currentRight = PositionsClass.OuttakePositionRight.Open;
            return new SequentialAction(t -> {
                motorControl.outTakeClawRight.setPosition(0.7);
                return false;
            },
                    new SleepAction(0.3));
        }
    }
    public class OutTakeArm {
        public Action Down() {
            return new SequentialAction(t -> {
                motorControl.outTakeArm.setPosition(0.65);
                return false;
            },
                    new SleepAction(0.3));
        }
        public Action Up() {
            return new SequentialAction(t -> {
                motorControl.outTakeArm.setPosition(0.3);
                return false;
            },
                    new SleepAction(0.3));
        }
    }
}