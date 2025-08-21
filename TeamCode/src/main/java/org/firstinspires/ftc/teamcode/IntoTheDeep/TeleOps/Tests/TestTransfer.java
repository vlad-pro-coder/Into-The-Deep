package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOps.Tests;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.EXTENSION_idle;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.EXTENSION_readytakesample;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.OVERHEAD_idle;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.OVERHEAD_preload;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Extendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;

@TeleOp
@Config
public class TestTransfer extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeFull(hardwareMap);
        RobotInitializers.InitializeForOperationTeleop();
        Lift.DoingAuto = true;
        Extendo.DoingAuto = true;
        Scheduler bigboy = new Scheduler();
        Scheduler tasks = new Scheduler();

        tasks
                .addTask(new Task() {
            @Override
            protected void Actions() {
                Extendo.state = Extendo.ExtendoStates.GOTOPOS;
                Extendo.setExtendoPos(400);
            }

            @Override
            protected boolean Conditions() {
                return Extendo.getPosition() > 50;
            }
        })
                .addTask(new Task() {
                            @Override
                            protected void Actions() {
                                Intake.DropDown();
                                Intake.RotateToStore();
                            }

                            @Override
                            protected boolean Conditions() {
                                return Intake.colorsensor.getDistance(DistanceUnit.CM) < 5;
                            }
                        })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Intake.Block();
                        Lift.state = Lift.LIFTSTATES.FREEWILL;
                        Lift.setLiftPos(50);
                        Extendo.state = Extendo.ExtendoStates.RETRACTING;
                        Intake.DropUp();
                        Outtake.setExtensionPos(EXTENSION_idle);
                        Outtake.setOverHeadPos(OVERHEAD_idle);
                    }

                    @Override
                    protected boolean Conditions() {
                        return Extendo.state != Extendo.ExtendoStates.RETRACTING;
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
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Lift.state = Lift.LIFTSTATES.RETRACTING;
                    }

                    @Override
                    protected boolean Conditions() {
                        return Lift.state != Lift.LIFTSTATES.RETRACTING;
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Outtake.setExtensionPos(EXTENSION_readytakesample);
                        Lift.CustomPowerToMotors(-0.6);
                    }

                    @Override
                    protected boolean Conditions() {
                        return Outtake.ExtensionDoneness();
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
                        Lift.state = Lift.LIFTSTATES.FREEWILL;
                        Lift.setLiftPos(400);
                        Outtake.setOverHeadPos(OVERHEAD_preload);
                        Intake.Unblock();
                    }

                    @Override
                    protected boolean Conditions() {
                        return Lift.getPosition() > 300;
                    }
                })

                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Outtake.setExtensionPos(1);
                    }

                    @Override
                    protected boolean Conditions() {
                        return Outtake.ExtensionDoneness() && Outtake.OverHeadDoneness();
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
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Outtake.setExtensionPos(EXTENSION_idle);
                        Outtake.setOverHeadPos(OVERHEAD_idle);
                    }

                    @Override
                    protected boolean Conditions() {
                        return Outtake.OverHeadDoneness(120);
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Outtake.closeClaw();
                    }

                    @Override
                    protected boolean Conditions() {
                        return Outtake.OverHeadDoneness(60);
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Lift.state = Lift.LIFTSTATES.RETRACTING;
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                });

        Outtake.openClaw();

        while(opModeInInit()){
            RobotInitializers.clearCache();

            Extendo.update();
            Lift.update();
            Outtake.update();

        }

        waitForStart();

        while(opModeIsActive()){
            RobotInitializers.clearCache();

            if(bigboy.IsSchedulerDone())
                bigboy.AddAnotherScheduler(tasks);

            Extendo.update();
            Lift.update();
            Outtake.update();
            bigboy.update();
        }
    }
}
