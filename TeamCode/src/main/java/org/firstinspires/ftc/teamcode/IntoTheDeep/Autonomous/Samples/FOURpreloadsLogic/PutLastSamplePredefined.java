package org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.FOURpreloadsLogic;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.CHASSIS_turntosubmersibleforcontinuostaking;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.EXTENSION_idle;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.EXTENSION_readytakesample;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.OVERHEAD_idle;

import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Extendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake;

public class PutLastSamplePredefined {
    public static Scheduler PutLastSamplePredefinedActions(double LiftOverbasket,double overbasketangle,double extensionoverbasket) {
        return new Scheduler()
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Chassis.setTargetPosition(CHASSIS_turntosubmersibleforcontinuostaking);
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Outtake.setExtensionPos(EXTENSION_readytakesample);
                        Lift.CustomPowerToMotors(-0.6);
                    }

                    @Override
                    protected boolean Conditions() {
                        return Outtake.ExtensionDoneness();
                    }
                })
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
                        Lift.state = Lift.LIFTSTATES.FREEWILL;
                        Lift.setLiftPos(LiftOverbasket);
                        Outtake.setExtensionPos(extensionoverbasket);
                        Outtake.setOverHeadPos(overbasketangle);
                        Intake.StopSpinner();
                        Outtake.closeClaw();
                    }

                    @Override
                    protected boolean Conditions() {
                        return Outtake.ExtensionDoneness() && Outtake.OverHeadDoneness(5) && Lift.IsLiftDone(150);
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
                        Extendo.state = Extendo.ExtendoStates.RETRACTING;
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                });
    }
}
