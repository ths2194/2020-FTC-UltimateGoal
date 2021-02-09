package org.firstinspires.ftc.teamcode.robot.vision;

public class VuforiaFeedback {
    private boolean seenTarget = false;
    private double x, y, angle;
    private String targetName;
    private final double angleOffset = -90;

    public void setSeenTarget(boolean seenTarget) {
        this.seenTarget = seenTarget;
    }

    public void setXY(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public void setAngle(double angle) {
        this.angle = angle+angleOffset;
    }

    public void setTargetName(String targetName) {
        this.targetName = targetName;
    }

    public boolean isSeenTarget() {
        return seenTarget;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getAngle() {
        return angle;
    }

    public String getTargetName() {
        return targetName;
    }
}
