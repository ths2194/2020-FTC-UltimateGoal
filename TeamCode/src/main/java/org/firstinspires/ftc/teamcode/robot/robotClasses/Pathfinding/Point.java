package org.firstinspires.ftc.teamcode.robot.robotClasses.Pathfinding;

public class Point implements Comparable<Point>{
    public int x, y;
    int i, j;
    double g,f; //cost variables for A* path finding calculation

    public Point(int i, int j, double g, double f) {
        this.x = Field.getGlobalX(j);
        this.y = Field.getGlobalY(i);
        this.i = i;
        this.j = j;
        this.g = g;
        this.f = f;
    }

    public int compareTo(Point that){
        return Double.compare(this.f,that.f);
    }

}