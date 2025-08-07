package org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.ApproachingSubmersibleActions;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.HEADING_infrontofsubmersible;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.PUREPERSUIT_pathtosubmersible;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.PUREPERSUIT_radius;

import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Chassis;

public class AfterThirdSampleGoToSubmersible {
    public static Scheduler AfterThirdSampleGoToSubmersibleActions = new Scheduler()
            .StartPurePersuit(PUREPERSUIT_pathtosubmersible,HEADING_infrontofsubmersible,PUREPERSUIT_radius)
            .addTask(new Task() {
                @Override
                protected void Actions() {

                }

                @Override
                protected boolean Conditions() {
                    return Chassis.IsPositionDone(30) && Chassis.IsHeadingDone(5);
                }
            });

}
