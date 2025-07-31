package org.firstinspires.ftc.teamcode.IntoTheDeep.ControllersRelated;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser;

public class SimplyfiedControllers extends Gamepad {
    private double LastCirclePressed,LastCrossPressed,LastSquarePressed,LastTrianglePressed;
    public static double TimeForIdentificationHolding = 0.1;
    public static Gamepad lastState;
    public SimplyfiedControllers(int gmID){//1 sau 2
        setGamepadId(gmID);
        lastState.copy(this);
    }

    public boolean triangleWasPressed(){
        return this.triangle && this.triangle != lastState.triangle;
    }
    public boolean squareWasPressed(){
        return this.square && this.square != lastState.square;
    }
    public boolean circleWasPressed(){
        return this.circle && this.circle != lastState.circle;
    }
    public boolean crossWasPressed(){
        return this.cross && this.cross != lastState.cross;
    }
    public boolean dpadUpWasPressed(){
        return this.dpad_up && this.dpad_up != lastState.dpad_up;
    }
    public boolean dpadDownWasPressed(){
        return this.dpad_down && this.dpad_down != lastState.dpad_down;
    }

    public boolean IsTriangleHeld(){
        return this.triangle && System.currentTimeMillis() * 1000 - LastTrianglePressed > TimeForIdentificationHolding;
    }
    public boolean IsSquareHeld(){
        return this.square && System.currentTimeMillis() * 1000 - LastSquarePressed > TimeForIdentificationHolding;
    }
    public boolean IsCircleHeld(){
        return this.circle && System.currentTimeMillis() * 1000 - LastCirclePressed > TimeForIdentificationHolding;
    }
    public boolean IsCrossHeld(){
        return this.cross && System.currentTimeMillis() * 1000 - LastCrossPressed > TimeForIdentificationHolding;
    }

    public void update(){//if update not called isHeld will always be true
        if(triangleWasPressed())
            LastTrianglePressed = System.currentTimeMillis() * 1000;
        if(squareWasPressed())
            LastSquarePressed = System.currentTimeMillis() * 1000;
        if(circleWasPressed())
            LastCirclePressed = System.currentTimeMillis() * 1000;
        if(crossWasPressed())
            LastCrossPressed = System.currentTimeMillis() * 1000;
        lastState.copy(this);
    }

}
