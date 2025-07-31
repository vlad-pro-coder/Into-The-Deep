package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOpLogic.SampleLogic;

import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake;

public class DropSampleAndRetract {

    public static Scheduler DropSampleAndRetractActions = new Scheduler()
            .addTask(new Task() {
                @Override
                protected void Actions() {
                    Outtake.openClaw();
                    Localizer.Reset();
                }

                @Override
                protected boolean Conditions() {
                    return Outtake.IsClawDone();
                }
            })
            .addTask(new Task() {
                @Override
                protected void Actions() {
                    Outtake.OverHead_TAKESAMPLE();
                    Outtake.setExtensionPos(0);
                }

                @Override
                protected boolean Conditions() {
                    return Outtake.OverHeadDoneness() && Localizer.getDistanceFromTwoPoints(null,Localizer.getCurrentPosition()) > 130;
                }
            })
            .addTask(new Task() {
                @Override
                protected void Actions() {
                    Lift.state = Lift.LIFTSTATES.RETRACTING;
                }

                @Override
                protected boolean Conditions() {
                    return Lift.getPosition() < 50;
                }
            });
}
