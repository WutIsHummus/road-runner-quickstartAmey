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



    public MotorActions(MotorControl motorControl) {
        this.motorControl = motorControl;
        this.slide = new Slide();
        this.intakeArm = new IntakeArm();
        this.flopper = new Flopper();
        this.IntakeArmServo = new IntakeArmServo();
        this.OutTakeLeft = new OutTakeLeft();
        this.OutTakeRight = new OutTakeRight();
        this.OutTakeArm = new OutTakeArm();
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
                IntakeArmServo.Fifth(),
                intakeArm.MidTransfer(),
                intakeArm.waitUntilFinished(),
                intakeArm.Intake(),
                intakeArm.waitUntilFinished(),
                new SleepAction(1),
                ClawOpen()
        );
    }

    public Action transfer(){
        currentIn = PositionsClass.IntakePosition.Transfer;
        return new SequentialAction(
                intakeArm.MidTransfer(),
                intakeArm.waitUntilFinished(),
                intakeArm.Transfer(),
                intakeArm.waitUntilFinished(),
                ClawClose()
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
     public Action temp() {
         return setTargetPosition(100);
     }
     public Action up() {
         return setTargetPosition(400);
     }
     public Action midPosition() {
         return setTargetPosition(350);
     }
     public Action backMidPosition() {
         return setTargetPosition(1100);
     }
    }
    public class IntakeArm {
        public Action setTargetPosition(double position) {
            return t -> {
                motorControl.intakeArm.setTargetPosition(position);
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
        public Action forceDown() {
            return setTargetPosition(-20);
        }
        public Action reset() {
            return new SequentialAction(t -> {motorControl.intakeArm.reset();return false;},
                    new SleepAction(0.1));
        }

        public Action Transfer() {
            return setTargetPosition(600);
        }
        public Action MidTransfer() {
            return setTargetPosition(250);
        }
        public Action Intake() {
            return setTargetPosition(0);
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
            return setTargetPosition(3000);
        }
    }
    public class IntakeArmServo {
        public Action Fifth() {
            return new SequentialAction(t -> {
                motorControl.intakeArmRight.setPosition(0.55);
                motorControl.intakeArmLeft.setPosition(0.45);
                return false;
            },
                    new SleepAction(0.3));
        }
        public Action Fourth() {
            return new SequentialAction(t -> {
                motorControl.intakeArmRight.setPosition(0.66);
                motorControl.intakeArmLeft.setPosition(0.50);
                return false;
            },
                    new SleepAction(0.3));
        }
        public Action Third() {
            return new SequentialAction(t -> {
                motorControl.intakeArmRight.setPosition(0.64);
                motorControl.intakeArmLeft.setPosition(0.505);
                return false;
            },
                    new SleepAction(0.3));
        }
        public Action Second() {
            return new SequentialAction(t -> {
                motorControl.intakeArmRight.setPosition(0.625);
                motorControl.intakeArmLeft.setPosition(0.510);
                return false;
            },
                    new SleepAction(0.3));
        }
        public Action Last() {
            return new SequentialAction(t -> {
                motorControl.intakeArmRight.setPosition(0.595);
                motorControl.intakeArmLeft.setPosition(0.515);
                return false;
            },
                    new SleepAction(0.3));
        }
        public Action Transfer() {
            return new SequentialAction(t -> {
                motorControl.intakeArmRight.setPosition(0.3);
                motorControl.intakeArmLeft.setPosition(0.56);

                return false;
            },
                    new SleepAction(0.3));
        }
        public Action Up() {
            return new SequentialAction(t -> {
                motorControl.intakeArmLeft.setPosition(0.4);
                motorControl.intakeArmRight.setPosition(0.2);

                return false;
            },
                    new SleepAction(0.3));
        }
    }

    public class OutTakeLeft {
        public Action Close() {
            currentOut = PositionsClass.OuttakePosition.Close;
            return new SequentialAction(t -> {
                motorControl.outTakeClawLeft.setPosition(0.6);
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
                motorControl.outTakeClawRight.setPosition(0.8);
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