package org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.FOURpreloadsLogic;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.EXTENSION_idle;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.EXTENSION_overbasket;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.EXTENSION_readytakesample;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.LIFT_highbasket;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.OVERHEAD_idle;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.OVERHEAD_overbasket;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.OVERHEAD_overbasketbeforeslam;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Extendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake;

public class SimultaniosSampleGrabingScoring {
    public static Scheduler SimultaniosSampleGrabingScoringActions(SparkFunOTOS.Pose2D PosToScoreSample, double ExtendoPos, double timeout){
        return new Scheduler()
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Chassis.setTargetPosition(PosToScoreSample);
                        Outtake.setExtensionPos(EXTENSION_readytakesample);
                    }

                    @Override
                    protected boolean Conditions() {
                        return Outtake.ExtensionDoneness();
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Outtake.closeClaw();
                    }

                    @Override
                    protected boolean Conditions() {
                        return Outtake.IsClawDone() && Chassis.IsPositionDone(150) && Chassis.IsHeadingDone(30);
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Lift.state = Lift.LIFTSTATES.FREEWILL;
                        Lift.setLiftPos(LIFT_highbasket);
                        Outtake.setOverHeadPos(OVERHEAD_overbasketbeforeslam);
                        Outtake.setExtensionPos(EXTENSION_overbasket);
                        Extendo.setExtendoPos(ExtendoPos-200);
                        Intake.Unblock();
                    }

                    @Override
                    protected boolean Conditions() {
                        return Extendo.IsExtendoDone(600);
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Intake.DropDown();
                        Intake.RotateToStore();
                    }

                    @Override
                    protected boolean Conditions() {
                        return Lift.IsLiftDone(60) && Outtake.OverHeadDoneness() && Outtake.ExtensionDoneness() && Chassis.IsPositionDone(30) && Chassis.IsHeadingDone(5);
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Outtake.setOverHeadPos(OVERHEAD_overbasket);
                    }

                    @Override
                    protected boolean Conditions() {
                        return Outtake.OverHeadDoneness(10);
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Outtake.openClaw();
                    }

                    @Override
                    protected boolean Conditions() {
                        return Outtake.IsClawDone();
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Outtake.setExtensionPos(EXTENSION_idle);
                        Outtake.setOverHeadPos(OVERHEAD_idle);
                        Extendo.setExtendoPos(ExtendoPos);
                    }

                    @Override
                    protected boolean Conditions() {
                        return Outtake.OverHeadDoneness(120);
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Lift.state = Lift.LIFTSTATES.RETRACTING;
                    }

                    @Override
                    protected boolean Conditions() {
                        return Extendo.IsExtendoDone(30);
                    }
                })
                .addTask(new Task() {
                    private long wait;
                    private long track;
                    @Override
                    protected void Actions() {
                        wait = (long) (timeout * 1000);
                        track = -1;
                    }

                    @Override
                    protected boolean Conditions() {
                        if (track == -1) {
                            track = System.currentTimeMillis();
                        }

                        boolean r = (System.currentTimeMillis() - track) >= wait;
                        if (r) track = -1;
                        return (Intake.HasMixedTeamPiece() && Intake.SampleReachedTrapDoor()) || r;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Intake.Block();
                        Extendo.state = Extendo.ExtendoStates.RETRACTING;
                        Intake.DropUp();
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .waitSeconds(0.2)
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Intake.RotateToEject();
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Intake.StopSpinner();
                    }

                    @Override
                    protected boolean Conditions() {
                        return Lift.getPosition() < 30 && Extendo.getPosition() < 30;
                    }
                });
    }
}
