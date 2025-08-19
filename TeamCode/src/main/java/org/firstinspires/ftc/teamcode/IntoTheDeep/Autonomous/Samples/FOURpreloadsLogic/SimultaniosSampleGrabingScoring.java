package org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.FOURpreloadsLogic;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.EXTENSION_idle;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.EXTENSION_readytakesample;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.OVERHEAD_idle;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Extendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake;

public class SimultaniosSampleGrabingScoring {
    public static Scheduler SimultaniosSampleGrabingScoringActions(SparkFunOTOS.Pose2D PosToScoreSample, double ExtendoPos, double timeout,double LiftOverbasket,double overbasketangle,double extensionoverbasket){
        return new Scheduler()
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Chassis.setTargetPosition(PosToScoreSample);
                        Outtake.setExtensionPos(EXTENSION_readytakesample);
                        Lift.CustomPowerToMotors(-0.5);
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
                        return Outtake.IsClawDone() && Chassis.IsPositionDone(250) && Chassis.IsHeadingDone(20);
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Lift.state = Lift.LIFTSTATES.FREEWILL;
                        Lift.setLiftPos(LiftOverbasket);
                        Outtake.setOverHeadPos(overbasketangle);
                        Intake.Unblock();
                    }

                    @Override
                    protected boolean Conditions() {
                        return Lift.getPosition() > 200;
                    }
                })

                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Outtake.setExtensionPos(extensionoverbasket);
                    }

                    @Override
                    protected boolean Conditions() {
                        return Chassis.IsPositionDone(100) && Chassis.IsHeadingDone(5);
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Extendo.setExtendoPos(ExtendoPos-200);
                    }

                    @Override
                    protected boolean Conditions() {
                        return Extendo.IsExtendoDone(650);
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Intake.DropDown();
                        //Intake.RotateToStore();
                    }

                    @Override
                    protected boolean Conditions() {
                        return Lift.IsLiftDone(150) && Outtake.OverHeadDoneness(5) && Outtake.ExtensionDoneness() && Chassis.IsPositionDone(80) && Chassis.IsHeadingDone(5);
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
                        Intake.RotateToStore();
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .waitSeconds(0.05)
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Outtake.setExtensionPos(EXTENSION_idle);
                        Outtake.setOverHeadPos(OVERHEAD_idle);
                        Extendo.setExtendoPos(ExtendoPos);
                        Outtake.closeClaw();
                    }

                    @Override
                    protected boolean Conditions() {
                        return Outtake.OverHeadDoneness(60);
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Lift.state = Lift.LIFTSTATES.RETRACTING;
                    }

                    @Override
                    protected boolean Conditions() {
                        return Extendo.IsExtendoDone(100);
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
                        return (Intake.HasMixedTeamPiece()) || r;
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
                        return Outtake.IsClawDone();
                    }
                })
                .waitSeconds(0.05)
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Intake.StopSpinner();
                    }

                    @Override
                    protected boolean Conditions() {
                        return Extendo.state != Extendo.ExtendoStates.RETRACTING;
                    }
                });
    }
}
