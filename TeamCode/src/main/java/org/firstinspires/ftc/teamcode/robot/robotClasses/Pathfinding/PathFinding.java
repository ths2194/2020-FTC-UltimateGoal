package org.firstinspires.ftc.teamcode.robot.robotClasses.Pathfinding;

import java.util.Arrays;
import java.util.PriorityQueue;
import java.util.Stack;

public class PathFinding {

    Field field = new Field();

    public Stack<Point> path = new Stack<>();

    public boolean calculatePath(int startX, int startY, int targetX, int targetY){
        //Runs A* pathfinding
        Point target = null;
        int startI = Field.getGridI(startY);
        int startJ = Field.getGridJ(startX);
        int targetI = Field.getGridI(targetY);
        int targetJ = Field.getGridJ(targetX);

        Stack<Point> newPath = new Stack<>();
        Point[][] pointBefore = new Point[144][144];
        boolean targetReached = false;

        PriorityQueue<Point> pq = new PriorityQueue<>();
        Point start = new Point(startI,startJ,0, calculateF(startI,startJ,targetI,targetJ,0));
        pq.add(start);
        boolean visited[][] = new boolean[144][144];
        double fValue[][] = new double[144][144];
        for (int i = 0; i<144; i++) Arrays.fill(fValue[i], Double.MAX_VALUE);
        fValue[startI][startJ] = start.f;
        while (!pq.isEmpty()){
            Point thisPoint = pq.poll();

            if (thisPoint.i == targetI && thisPoint.j == targetJ){
//                newPath.add(thisPoint);
                target = thisPoint;
                targetReached = true;
                break;
            }

            if (visited[thisPoint.i][thisPoint.j]) continue;
            visited[thisPoint.i][thisPoint.j] = true;

            for (int di = -1; di<=1; di++){
                for (int dj = -1; dj<=1; dj++){
                    if (thisPoint.i+di >=144 || thisPoint.i+di <0 || thisPoint.j+dj>=144 || thisPoint.j+dj<0) continue;
                    if (visited[thisPoint.i+di][thisPoint.j+dj]) continue;
                    if (!field.isOpenWithGridAt(thisPoint.i+di, thisPoint.j+dj)) continue;
                    double dist;
                    if (di == 0 || dj == 0) dist = D;
                    else dist = D2;
                    double nextF = calculateF(thisPoint.i+di,thisPoint.j+dj,targetI,targetJ,thisPoint.g+dist);
                    if (nextF < fValue[thisPoint.i+di][thisPoint.j+dj]){
                        fValue[thisPoint.i+di][thisPoint.j+dj] = nextF;
                        Point nextPoint = new Point(thisPoint.i+di,thisPoint.j+dj, thisPoint.g+dist, nextF);
                        pq.add(nextPoint);
                        pointBefore[thisPoint.i+di][thisPoint.j+dj] = thisPoint;
                    }

                }
            }
        }

        if (!targetReached) return false;

        int direction = -1;
        Point lastPoint = target;
        int currI = targetI; int currJ = targetJ;
        while (!(currI==startI && currJ==startJ)){
            //adds points to path
            int newDirection = calculateDirection(lastPoint,pointBefore[currI][currJ]);
            if (newDirection != direction){
                direction = newDirection;
                newPath.add(lastPoint);
            }
            lastPoint = pointBefore[currI][currJ];
            currI = lastPoint.i;
            currJ = lastPoint.j;
        }

        path = newPath;

        return true;

    }

    static int calculateDirection (Point a, Point pointBeforeA){
        if (pointBeforeA.x==a.x && pointBeforeA.y>a.y) return 0;
        if (pointBeforeA.x>a.x && pointBeforeA.y==a.y) return 1;
        if (pointBeforeA.x==a.x && pointBeforeA.y<a.y) return 2;
        if (pointBeforeA.x<a.x && pointBeforeA.y==a.y) return 3;
        if (pointBeforeA.x>a.x && pointBeforeA.y>a.y) return 4;
        if (pointBeforeA.x>a.x && pointBeforeA.y<a.y) return 5;
        if (pointBeforeA.x<a.x && pointBeforeA.y<a.y) return 6;
        if (pointBeforeA.x<a.x && pointBeforeA.y>a.y) return 7;
        return -1;
    }

    static int D = 1; static double D2 = Math.sqrt(2);
    static double calculateH(int x, int y, int targetX, int targetY){
        //Octile distance, estimated cost
        int dx = Math.abs(x - targetX);
        int dy = Math.abs(y - targetY);
        return D * (dx + dy) + (D2 - 2 * D) * Math.min(dx, dy);
    }

    static double calculateF(int x, int y, int targetX, int targetY, double g){
        //g is cost from start node
        return g+calculateH(x, y, targetX,targetY);
    }





}
