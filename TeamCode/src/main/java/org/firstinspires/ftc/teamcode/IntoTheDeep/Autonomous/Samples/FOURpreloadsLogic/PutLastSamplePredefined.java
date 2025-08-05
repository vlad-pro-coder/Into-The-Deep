package org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.FOURpreloadsLogic;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.EXTENSION_idle;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.EXTENSION_overbasket;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.EXTENSION_readytakesample;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.LIFT_highbasket;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.OVERHEAD_idle;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.OVERHEAD_overbasket;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.OVERHEAD_overbasketbeforeslam;

import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake;

public class PutLastSamplePredefined {
    public static Scheduler PutLastSamplePredefinedActions = new Scheduler()
            .addTask(new Task() {
                @Override
                protected void Actions() {
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
                    return Outtake.IsClawDone();
                }
            })
            .addTask(new Task() {
                @Override
                protected void Actions() {
                    Lift.state = Lift.LIFTSTATES.FREEWILL;
                    Lift.setLiftPos(LIFT_highbasket);
                    Outtake.setExtensionPos(EXTENSION_overbasket);
                    Outtake.setOverHeadPos(OVERHEAD_overbasketbeforeslam);
                    Intake.StopSpinner();
                }

                @Override
                protected boolean Conditions() {
                    return Outtake.ExtensionDoneness() && Outtake.OverHeadDoneness() && Lift.IsLiftDone(30);
                }
            })
            .addTask(new Task() {
                @Override
                protected void Actions() {
                    Outtake.setOverHeadPos(OVERHEAD_overbasket);
                }

                @Override
                protected boolean Conditions() {
                    return Outtake.OverHeadDoneness();
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
                    Outtake.setOverHeadPos(OVERHEAD_idle);
                    Outtake.setExtensionPos(EXTENSION_idle);
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
                    return Lift.getPosition() < 30;
                }
            });
}
