package org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.ApproachingSubmersibleActions;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.EXTENSION_idle;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.EXTENSION_overbasket;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.EXTENSION_readytakesample;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.HEADING_fromsubmersibletobasket;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.HEADING_infrontofsubmersible;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.LIFT_high;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.LIFT_midway;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.OVERHEAD_fromsubtobasket;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.OVERHEAD_idle;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.PUREPERSUIT_pathbacktobasket;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.PUREPERSUIT_pathtosubmersible;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.PUREPERSUIT_radius;

import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Extendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake;

public class TakeSubmersibleSampleToHighBasket {
    public static Scheduler GoingBackAndForthToHighBasketAndSubmersibleActions() {
        return new Scheduler()
                .StartPurePersuit(PUREPERSUIT_pathbacktobasket, HEADING_fromsubmersibletobasket, PUREPERSUIT_radius)
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Extendo.state = Extendo.ExtendoStates.RETRACTING;
                        Intake.RotateToStore();
                        Intake.DropUp();
                    }

                    @Override
                    protected boolean Conditions() {
                        return Intake.SampleReachedTrapDoor();
                    }
                })
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
                        Intake.RotateToEject(0.6);
                    }

                    @Override
                    protected boolean Conditions() {
                        return Extendo.getPosition() < 30;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Lift.CustomPowerToMotors(-0.5);
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
                        Intake.StopSpinner();
                    }

                    @Override
                    protected boolean Conditions() {
                        return Outtake.IsClawDone();
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Lift.state = Lift.LIFTSTATES.FREEWILL;
                        Lift.setLiftPos(LIFT_midway);
                        Intake.Unblock();
                        Outtake.setOverHeadPos(OVERHEAD_fromsubtobasket);
                    }

                    @Override
                    protected boolean Conditions() {
                        return Chassis.IsHeadingDone(30) && Chassis.IsPositionDone(900);
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Extendo.state = Extendo.ExtendoStates.BALANCEFROMPOINT;
                        Lift.state = Lift.LIFTSTATES.FREEWILL;
                        Lift.setLiftPos(LIFT_high);
                        Outtake.setExtensionPos(EXTENSION_overbasket);
                    }

                    @Override
                    protected boolean Conditions() {
                        return Lift.IsLiftDone(300) && Outtake.OverHeadDoneness() && Outtake.ExtensionDoneness() && Chassis.IsHeadingDone(5) && Chassis.IsPositionDone(80) && Localizer.getVelocity().h < Math.toRadians(5) && Localizer.getVelocity().x < 70 && Localizer.getVelocity().y < 70;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Outtake.openClaw();
                        Intake.DropUp();
                        Extendo.state = Extendo.ExtendoStates.RETRACTING;
                    }

                    @Override
                    protected boolean Conditions() {
                        return Outtake.IsClawDone();
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Chassis.Heading.setPidCoefficients(Chassis.ToSubermsibleHeading);
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .StartPurePersuit(PUREPERSUIT_pathtosubmersible, HEADING_infrontofsubmersible, PUREPERSUIT_radius)
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Outtake.setExtensionPos(EXTENSION_idle);
                        Outtake.setOverHeadPos(OVERHEAD_idle);
                    }

                    @Override
                    protected boolean Conditions() {
                        return Outtake.OverHeadDoneness(60);
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Outtake.closeClaw();
                    }

                    @Override
                    protected boolean Conditions() {
                        return Outtake.IsClawDone();
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Lift.state = Lift.LIFTSTATES.RETRACTING;
                    }

                    @Override
                    protected boolean Conditions() {
                        return Lift.getPosition() < 30;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Outtake.openClaw();
                    }

                    @Override
                    protected boolean Conditions() {
                        return Chassis.IsPositionDone(50) && Chassis.IsHeadingDone(5) && Localizer.getVelocity().h < Math.toRadians(3);
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Chassis.Heading.setPidCoefficients(Chassis.NormalHeading);
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                });
    }
}

