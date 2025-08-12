package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOpLogic.PanicActions;

import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;

public class PanicElevatorPos {

    public static Scheduler PanicElevatorUpActions() {
        return new Scheduler()
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Lift.state = Lift.LIFTSTATES.FREEWILL;
                        Lift.setLiftPos(Lift.HighBasketPos);
                    }

                    @Override
                    protected boolean Conditions() {
                        return Lift.IsLiftDone(30);
                    }
                });
    }
}
