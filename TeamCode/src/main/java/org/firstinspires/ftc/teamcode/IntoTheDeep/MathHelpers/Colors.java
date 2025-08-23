package org.firstinspires.ftc.teamcode.IntoTheDeep.MathHelpers;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Colors {
    public static class Color{
        public double r, g, b, d;
        public Color(double r, double g, double b, double d){
            this.r = r;
            this.g = g;
            this.b = b;
            this.d = d;
        }

    }
    public enum ColorType {
        BLUE(new Color(0.3, 0.5, 1.2, 2.9)),
        BLUE1(new Color(0.1, 0.3, 0.5, 4.7)),



        RED(new Color(1.5, 0.7, 0.3, 2.4)),
        RED1(new Color(0.8, 0.4, 0.2,3.3)),
        RED2(new Color(0.5, 0.3, 0.2, 4.4)),
        RED3(new Color(2.8, 1.6, 0.9, 1.3)),

        YELLOW(new Color(2.3, 2.8, 0.6, 2.4)),
        YELLOW1(new Color(0.8, 1, 0.2, 4.4)),
        YELLOW2(new Color(1.7, 2.1, 0.4, 3)),
        YELLOW3(new Color(0.5, 0.6, 0.2, 5.8)),

        NONE(new Color(0.2, 0.3, 0.2, 4));

        private final Color color;

        ColorType(Color c) {
            this.color = c;
        }

        public Color getColor(){ return color; }

    }
    public static double getColorDistance(Color c1, Color c2) {
        double rDiff = c1.r - c2.r;
        double gDiff = c1.g - c2.g;
        double bDiff = c1.b - c2.b;
        return Math.sqrt(rDiff * rDiff + gDiff * gDiff + bDiff * bDiff);
    }

    public static ColorType getColorFromRGB(Color input) {
        double redDist2 = Math.min(getColorDistance(input, ColorType.RED.getColor()), getColorDistance(input, ColorType.RED3.getColor()));
        double redDist1 = Math.min(getColorDistance(input, ColorType.RED.getColor()), getColorDistance(input, ColorType.RED1.getColor()));
        double redDist = Math.min(getColorDistance(input, ColorType.RED2.getColor()), Math.min(redDist1,redDist2));
        double blueDist = Math.min(getColorDistance(input, ColorType.BLUE.getColor()), getColorDistance(input, ColorType.BLUE1.getColor()));
        double yellowDist2 = Math.min(getColorDistance(input, ColorType.YELLOW.getColor()), getColorDistance(input, ColorType.YELLOW3.getColor()));
        double yellowDist1 = Math.min(getColorDistance(input, ColorType.YELLOW.getColor()), getColorDistance(input, ColorType.YELLOW1.getColor()));
        double yellowDist = Math.min(getColorDistance(input, ColorType.YELLOW2.getColor()), Math.min(yellowDist2,yellowDist1));
        if(redDist < blueDist && redDist < yellowDist) return ColorType.RED;
        if(blueDist < redDist && blueDist < yellowDist) return ColorType.BLUE;
        return ColorType.YELLOW;
    }

}