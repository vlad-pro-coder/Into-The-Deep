package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOpLogic.SampleLogic;


import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Extendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake;

public class TransferSample {

    public static Scheduler TransferSampleActions(){
        return new Scheduler()
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Outtake.openClaw();
                        Outtake.setExtensionPos(0);
                        Outtake.OverHead_TAKESAMPLE();
                        Intake.RotateToStore();
                        Intake.DropUp();
                        Lift.state = Lift.LIFTSTATES.FREEWILL;
                        Lift.setLiftPos(30);
                    }

                    @Override
                    protected boolean Conditions() {
                        return Intake.SampleReachedTrapDoor();
                    }
                })
                .waitSeconds(0.1)
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Intake.Block();
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .waitSeconds(0.1)
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Intake.RotateToEject();
                        Extendo.state = Extendo.ExtendoStates.RETRACTING;
                    }

                    @Override
                    protected boolean Conditions() {
                        return Extendo.state != Extendo.ExtendoStates.RETRACTING;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Lift.state = Lift.LIFTSTATES.RETRACTING;
                    }

                    @Override
                    protected boolean Conditions() {
                        return Lift.state != Lift.LIFTSTATES.RETRACTING;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Lift.CustomPowerToMotors(-0.5);
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .waitSeconds(0.15)
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Intake.StopSpinner();
                        Outtake.setExtensionPos(0.3);
                    }

                    @Override
                    protected boolean Conditions() {
                        return Outtake.OverHeadDoneness();
                    }
                })
                .waitSeconds(0.1)
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Outtake.closeClawTight();
                    }
                    @Override
                    protected boolean Conditions() {
                        return Outtake.IsClawDone();
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Lift.state = Lift.LIFTSTATES.OFF;
                        Intake.Unblock();
                    }
                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                });
    }
}
