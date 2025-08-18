package org.firstinspires.ftc.teamcode.IntoTheDeep.Climb;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Climb.ClimbConstants.BAR1;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Climb.ClimbConstants.BAR2;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Climb.ClimbConstants.climbArmIntertia;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Climb.ClimbConstants.pitch;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Climb.ClimbConstants.tasks;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Climb.ClimbConstants.time;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Extendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.PtoAndWheelie;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;

public class Climb {
    public static Scheduler ClimbActions(){
        return new Scheduler()
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Lift.state = Lift.LIFTSTATES.OFF;
                        PtoAndWheelie.TiltRobot();
                        PtoAndWheelie.disengagePTO();
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Lift.state = Lift.LIFTSTATES.FREEWILL;
                        Lift.setLiftPos(BAR1 + 150);
                    }

                    @Override
                    protected boolean Conditions() {
                        return Lift.getPosition() >= BAR1 + 50;
                    }
                })
                .waitSeconds(0.3)
                .addTask(new Task() {

                    @Override
                    protected void Actions() {
                        Lift.setLiftPos(BAR1 + 80);
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        PtoAndWheelie.engagePTO();
                        PtoAndWheelie.powerToChassis(-0.2);
//                        Elevator.setTargetPosition(BAR1 + 60);
                        Lift.CustomPowerToMotors(-0.5);
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })

                .waitSeconds(0.1)
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Lift.state = Lift.LIFTSTATES.OFF;
                        PtoAndWheelie.setPosChassisDrivenLift(-20);
                    }

                    @Override
                    protected boolean Conditions() {
                        PtoAndWheelie.UpdateChassisDrivenLift();
                        return Lift.getPosition() < 5;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        PtoAndWheelie.powerToChassis(0);
                        PtoAndWheelie.IdleWheeliePos();
                        time.reset();
                    }

                    @Override
                    protected boolean Conditions() {
                        return Lift.getPosition() >= 50 || time.seconds() >= 0.6;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        PtoAndWheelie.disengagePTO();
                        Lift.state = Lift.LIFTSTATES.FREEWILL;
                        Lift.setLiftPos(BAR2);
                    }

                    @Override
                    protected boolean Conditions() {
                        if(pitch > 3 && (Lift.getPosition() <= BAR2 - 30 && Lift.getPosition() >= 250))
                            Lift.state = Lift.LIFTSTATES.OFF;
                        else
                            Lift.state = Lift.LIFTSTATES.FREEWILL;
                        return Lift.getPosition() >= BAR2 - 20;
                    }
                })
                .addTask(new Task() {     ///pula pula pula pula pula pula pula pula pula pula pula pula pula pula pula pula
                    @Override
                    protected void Actions() {
                        Lift.state = Lift.LIFTSTATES.FREEWILL;
                        Outtake.setOverHeadPos(climbArmIntertia);
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {

                    }

                    @Override
                    protected boolean Conditions() {
                        if(pitch > -3){
                            Lift.setLiftPos(BAR2 - 250);
                            return true;
                        }
                        return false;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {

                    }

                    @Override
                    protected boolean Conditions() {
                        return Lift.getPosition() < BAR2 - 100;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        PtoAndWheelie.engagePTO();
                        PtoAndWheelie.powerToChassis(-0.4);
                        Lift.CustomPowerToMotors(0.4);
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .waitSeconds(0.1)
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Lift.state = Lift.LIFTSTATES.OFF;
                        PtoAndWheelie.setPosChassisDrivenLift(-100);
                    }

                    @Override
                    protected boolean Conditions() {
                        PtoAndWheelie.UpdateChassisDrivenLift();
                        return Lift.getPosition() < 10;
                    }
                })
                .addTask(new Task() {

                    @Override
                    protected void Actions() {
                        PtoAndWheelie.powerToChassis(0);
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
        ;

    }
    public static void updateClimb(){
        pitch = RobotInitializers.imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
        RobotLog.ii("pitch","" + pitch);
        Extendo.CustomPowerToMotors(-0.6);
        tasks.update();

        Lift.update();
        Outtake.update();
        Extendo.update();
    }
}
