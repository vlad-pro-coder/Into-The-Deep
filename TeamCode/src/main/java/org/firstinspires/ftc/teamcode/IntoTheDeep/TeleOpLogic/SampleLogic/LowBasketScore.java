package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOpLogic.SampleLogic;

import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake;

public class LowBasketScore {

    public static Scheduler LowBasketScoreActions = new Scheduler()
            .addTask(new Task() {
                @Override
                protected void Actions() {
                    Lift.state = Lift.LIFTSTATES.LOWBASKET;
                    Outtake.OverHead_BASKETMOVINGSAFEPOS();
                }

                @Override
                protected boolean Conditions() {
                    return Math.abs(Lift.getPosition() - Lift.LowBasketPos) < 40 && Outtake.OverHeadDoneness();
                }
            })
            .addTask(new Task() {
                @Override
                protected void Actions() {
                    Outtake.OverHead_OVERBASKET();
                }

                @Override
                protected boolean Conditions() {
                    return Outtake.OverHeadDoneness();
                }
            });
}
