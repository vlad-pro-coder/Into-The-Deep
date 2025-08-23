package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOpLogic.SampleLogic;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake;

public class DropSampleAndRetract {
    public static SparkFunOTOS.Pose2D lastpos = new SparkFunOTOS.Pose2D(0,0,0);

    public static Scheduler DropSampleAndRetractActions(){
        return new Scheduler()
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Outtake.openClaw();
                        lastpos = Localizer.getCurrentPosition();
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
                        return Outtake.OverHeadDoneness() && Localizer.getDistanceFromTwoPoints(lastpos,Localizer.getCurrentPosition()) > 80;
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
                        return Lift.getPosition() < 30;
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
                });
    }
}
