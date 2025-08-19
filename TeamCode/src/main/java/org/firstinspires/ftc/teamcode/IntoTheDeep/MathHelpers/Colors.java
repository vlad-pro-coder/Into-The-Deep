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
        BLUE(new Color(0.37, 0.72, 1.6, 2.6)),
        BLUE1(new Color(0.14, 0.29, 0.52, 5.1)),

        RED(new Color(0.41, 0.28, 0.16, 5.22)),
        RED1(new Color(0.82, 0.44, 0.23,3.2)),
        RED2(new Color(1.69, 0.82, 0.41, 2)),

        YELLOW(new Color(0.6, 0.74, 0.21, 5.25)),
        YELLOW1(new Color(1.27, 1.33, 0.32, 4)),
        YELLOW2(new Color(2.7, 3, 0.63, 2)),

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
        double redDist1 = Math.min(getColorDistance(input, ColorType.RED.getColor()), getColorDistance(input, ColorType.RED1.getColor()));
        double redDist = Math.min(getColorDistance(input, ColorType.RED2.getColor()), redDist1);
        double blueDist = Math.min(getColorDistance(input, ColorType.BLUE.getColor()), getColorDistance(input, ColorType.BLUE1.getColor()));
        double yellowDist1 = Math.min(getColorDistance(input, ColorType.YELLOW.getColor()), getColorDistance(input, ColorType.YELLOW1.getColor()));
        double yellowDist = Math.min(getColorDistance(input, ColorType.YELLOW2.getColor()), yellowDist1);
        if(redDist < blueDist && redDist < yellowDist) return ColorType.RED;
        if(blueDist < redDist && blueDist < yellowDist) return ColorType.BLUE;
        return ColorType.YELLOW;
    }

}