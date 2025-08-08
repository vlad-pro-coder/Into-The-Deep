package org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.ApproachingSubmersibleActions;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.HEADING_infrontofsubmersible;

import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Extendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Intake;

public class FailedToDetect {
    public static Scheduler FailedToDetectActions = new Scheduler()
            .addTask(new Task() {
                @Override
                protected void Actions() {
                    Intake.DropUp();
                    Intake.RotateToEject();
                    Extendo.state = Extendo.ExtendoStates.RETRACTING;
                    Chassis.setHeading(HEADING_infrontofsubmersible);
                }

                @Override
                protected boolean Conditions() {
                    return Extendo.getPosition() < 30 && Chassis.IsHeadingDone(5);
                }
            });
}
