package org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.ApproachingSubmersibleActions;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.HEADING_infrontofsubmersible;

import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Extendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Localizer;

public class FailedToDetect {
    public static Scheduler FailedToDetectActions() {
        return new Scheduler()
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Intake.DropUp();
                        Intake.RotateToEject(0.4);
                        Extendo.state = Extendo.ExtendoStates.RETRACTING;
                        Intake.Unblock();
                        Chassis.setHeading(HEADING_infrontofsubmersible);
                    }

                    @Override
                    protected boolean Conditions() {
                        return Extendo.getPosition() < 30 && Chassis.IsHeadingDone(4) && Localizer.getVelocity().h < Math.toRadians(2);
                    }
                });
    }
}
