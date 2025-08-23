package org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.ApproachingSubmersibleActions;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.EXTENSION_idle;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.HEADING_infrontofsubmersible;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.LIFT_firstsample;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.OVERHEAD_idle;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.PUREPERSUIT_pathtosubmersible;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.PUREPERSUIT_radius;

import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake;

public class AfterThirdSampleGoToSubmersible {
    public static Scheduler AfterThirdSampleGoToSubmersibleActions() {
        return new Scheduler()
                /*.addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Chassis.Heading.setPidCoefficients(Chassis.ToSubermsibleHeading);
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })*/
                .StartPurePersuit(PUREPERSUIT_pathtosubmersible, HEADING_infrontofsubmersible, PUREPERSUIT_radius)
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
                        return Chassis.IsPositionDone(50) && Chassis.IsHeadingDone(5);
                    }
                });
    }
}
