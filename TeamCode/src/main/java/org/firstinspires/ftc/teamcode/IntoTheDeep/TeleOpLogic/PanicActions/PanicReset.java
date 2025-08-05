package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOpLogic.PanicActions;

import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Extendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake;

public class PanicReset {

    public static Scheduler PanicResetActions = new Scheduler()
            .addTask(new Task() {
                @Override
                protected void Actions() {
                    Lift.state = Lift.LIFTSTATES.OFF;
                    Outtake.openClaw();
                    Extendo.state = Extendo.ExtendoStates.FREEWILL;
                    Outtake.OverHead_TAKESAMPLE();
                    Outtake.setExtensionPos(0);
                }

                @Override
                protected boolean Conditions() {
                    return Outtake.IsClawDone() &&
                            Outtake.OverHeadDoneness() &&
                            Outtake.ExtensionDoneness(0);
                }
            });

}
