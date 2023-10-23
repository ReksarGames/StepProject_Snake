package com.codenjoy.dojo.snake.client;

/*-
 * #%L
 * Codenjoy - it's a dojo-like platform from developers to developers.
 * %%
 * Copyright (C) 2018 Codenjoy
 * %%
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/gpl-3.0.html>.
 * #L%
 */


import com.codenjoy.dojo.client.Solver;
import com.codenjoy.dojo.client.WebSocketRunner;
import com.codenjoy.dojo.services.Dice;
import com.codenjoy.dojo.services.Direction;
import com.codenjoy.dojo.services.RandomDice;
import com.codenjoy.dojo.services.Point;

import java.util.*;


/**
 * User: your name
 */
public class YourSolver implements Solver<Board> {

    private Dice dice;
    private Board board;

    public YourSolver(Dice dice) {
        this.dice = dice;
    }
// Переопределяется метод GET для управления. Обновляется каждую секунду
    @Override
    public String get(Board board) {
        this.board = board;
        System.out.println(board.toString());
        if (board.isGameOver()){return Direction.RIGHT.toString();}

        Direction safeDirection = avoidObstacles(board);

        // Получите координаты головы змейки
        Point head = board.getHead();
        // Получите координаты яблока
        List<Point> apples = board.getApples();

        // Получите координаты камней
        List<Point> stones = board.getStones();
        List<Point> walls = board.getWalls(); // Стены

        Direction findFastestRouteToApples = findFastestRouteToApples(board);

        return safeDirection.toString();
    }

    private Direction avoidObstacles(Board board){
        Direction currentDirection = board.getSnakeDirection();
        Point head = board.getHead();
        Point nextHead = currentDirection.change(head);

        if (board.isValid(nextHead) && !board.getBarriers().contains(nextHead)) {
            // Текущее направление безопасно, используем его.
            return currentDirection;
        } else {
            // Попробуем альтернативные направления.
            for (Direction alternativeDirection : Direction.values()) {
                if (alternativeDirection != currentDirection) {
                    Point alternativeNextHead = alternativeDirection.change(head);
                    if (board.isValid(alternativeNextHead) && !board.getBarriers().contains(alternativeNextHead)) {
                        // Найдено безопасное альтернативное направление.
                        return alternativeDirection;
                    }
                }
            }
        }
        return currentDirection;
    }
    private Direction findFastestRouteToApples(Board board) {
        Point head = board.getHead();
        List<Point> apples = board.getApples();
        List<Point> barriers = board.getBarriers();
        if (apples.isEmpty()) {
            // Если нет доступных яблок, вернем безопасное направление.
            return avoidObstacles(board);
        }
        Point nearestApple = findNearestApple(head, apples);
        List<Point> path = findPathAStar(board, head, nearestApple, barriers);
        if (path != null && !path.isEmpty()) {
            // Получите направление для следующего шага по пути.
            Point nextStep = path.get(1);  // Первая точка пути - текущее положение змейки.
            return getDirectionTowardsPoint(head, nextStep); // TODO: убери
        }
        return avoidObstacles(board);
    }
    private Direction getDirectionTowardsPoint(Point from, Point to) {
        int fromX = from.getX();
        int fromY = from.getY();
        int toX = to.getX();
        int toY = to.getY();

        // Рассчитываем разницу между координатами точек.
        int dx = toX - fromX;
        int dy = toY - fromY;

        // Определяем направление в зависимости от разницы координат.
        if (dx > 0) {
            return Direction.RIGHT;
        } else if (dx < 0) {
            return Direction.LEFT;
        } else if (dy > 0) {
            return Direction.DOWN;
        } else if (dy < 0) {
            return Direction.UP;
        }

        // Если разница равна нулю, то точки совпадают, и мы можем вернуть любое направление (например, UP).
        return Direction.UP;
    }

    private Point findNearestApple(Point head, List<Point> apples) {
        Point nearestApple = null;
        int minDistance = Integer.MAX_VALUE;

        for (Point apple : apples) {
            int distance = Math.abs(head.getX() - apple.getX()) + Math.abs(head.getY() - apple.getY());
            if (distance < minDistance) {
                minDistance = distance;
                nearestApple = apple;
            }
        }

        return nearestApple;
    }

    private List<Point> findPathAStar(Board board, Point start, Point target, List<Point> obstacles) {
        // Создаем структуры данных для алгоритма A*.
        PriorityQueue<Node> openSet = new PriorityQueue<>();
        Set<Point> closedSet = new HashSet<>();

        // Создаем начальную вершину с координатами стартовой точки.
        Node initialNode = new Node(start, null, 0, target.distance(start));
        openSet.add(initialNode);

        while (!openSet.isEmpty()) {
            // Извлекаем вершину с наименьшей оценкой f из openSet.
            Node current = openSet.poll();

            if (current.position.equals(target)) {
                // Найден путь, строим его и возвращаем.
                return buildPath(current);
            }

            closedSet.add(current.position);

            for (Direction direction : Direction.values()) {
                Point neighborPos = direction.change(current.position);

                if (!board.isValid(neighborPos) || closedSet.contains(neighborPos) || obstacles.contains(neighborPos)) {
                    continue; // Пропускаем недопустимые соседние точки.
                }

                double gScore = current.gScore + 1; // Предполагается, что стоимость движения между соседними точками = 1.

                Node neighbor = new Node(neighborPos, current, gScore, target.distance(neighborPos));

                if (!openSet.contains(neighbor) || gScore < neighbor.gScore) {
                    openSet.add(neighbor);
                }
            }
        }

        // Путь не найден, возвращаем null.
        return null;
    }

    // Вспомогательный метод для построения пути от цели к старту.
    private List<Point> buildPath(Node node) {
        List<Point> path = new ArrayList<>();
        Node current = node;
        while (current != null) {
            path.add(current.position);
            current = current.parent;
        }
        Collections.reverse(path);
        return path;
    }

    // Внутренний класс для представления вершины графа (позиции).
    private static class Node implements Comparable<Node> {
        Point position;
        Node parent;
        double gScore; // Суммарная стоимость пути от старта до этой вершины.
        double fScore; // Оценка f = g + h, где h - эвристическая оценка до цели.

        Node(Point position, Node parent, double gScore, double hScore) {
            this.position = position;
            this.parent = parent;
            this.gScore = gScore;
            this.fScore = gScore + hScore;
        }

        @Override
        public int compareTo(Node other) {
            return Double.compare(this.fScore, other.fScore);
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj) return true;
            if (obj == null || getClass() != obj.getClass()) return false;
            Node other = (Node) obj;
            return position.equals(other.position);
        }

        @Override
        public int hashCode() {
            return Objects.hash(position);
        }
    }

    public static void main(String[] args) {
        WebSocketRunner.runClient(
                // paste here board page url from browser after registration
                "http://157.230.104.223/codenjoy-contest/board/player/r6u98npcep9axkeejoo9?code=1941180044037278472",
                new YourSolver(new RandomDice()),
                new Board());
    }

}
