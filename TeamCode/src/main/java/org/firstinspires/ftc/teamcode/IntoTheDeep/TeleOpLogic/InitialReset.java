package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOpLogic;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.ParkingActions.ParkingConstants.OVERHEAD_forparking;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake.OverHeadTakeSampPos;

import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Extendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.PtoAndWheelie;

public class InitialReset {
    public static Scheduler InitialResetNotParked(){
        return new Scheduler()
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Extendo.state = Extendo.ExtendoStates.FREEWILL;
                        PtoAndWheelie.disengagePTO();
                        PtoAndWheelie.IdleWheeliePos();
                        Outtake.armProfile.setInstant(OverHeadTakeSampPos-1);
                        Outtake.OverHead_TAKESAMPLE();
                        Outtake.setExtensionPos(0);
                        Outtake.closeClaw();
                        Intake.DropUp();
                        Intake.Unblock();
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
                        Outtake.openClaw();
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                });
    }

    public static Scheduler InitialResetParked() {
        return new Scheduler()
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        PtoAndWheelie.disengagePTO();
                        PtoAndWheelie.IdleWheeliePos();
                        Lift.state = Lift.LIFTSTATES.FREEWILL;
                        Lift.setLiftPos(450);
                        Extendo.state = Extendo.ExtendoStates.FREEWILL;
                        Outtake.setOverHeadPos(OVERHEAD_forparking-40);
                    }

                    @Override
                    protected boolean Conditions() {
                        return Lift.IsLiftDone(30) && Outtake.OverHeadDoneness();
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Outtake.setExtensionPos(0);
                    }

                    @Override
                    protected boolean Conditions() {
                        return Outtake.ExtensionDoneness();
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Outtake.OverHead_TAKESAMPLE();
                        Outtake.closeClaw();
                        Intake.DropUp();
                        Intake.Unblock();
                    }

                    @Override
                    protected boolean Conditions() {
                        return Outtake.ExtensionDoneness() && Outtake.OverHeadDoneness();
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
                        Outtake.openClaw();
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                });
    }
}
