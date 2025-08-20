package org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.ParkingActions;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.ParkingActions.ParkingConstants.LIFT_forparking;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.ParkingActions.ParkingConstants.OVERHEAD_forparking;

import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake;

public class Parking {
    public static Scheduler ParkingActions() {
        return new Scheduler()
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Lift.state = Lift.LIFTSTATES.FREEWILL;
                        Lift.setLiftPos(LIFT_forparking);
                        Outtake.setOverHeadPos(OVERHEAD_forparking);
                    }

                    @Override
                    protected boolean Conditions() {
                        return Outtake.OverHeadDoneness(40);
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Outtake.setExtensionPos(1);
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                });

    }
}
