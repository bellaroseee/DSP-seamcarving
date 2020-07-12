package seamcarving;

import astar.WeightedEdge;
import edu.princeton.cs.algs4.Picture;
import pq.ExtrinsicMinPQ;
import pq.TreeMapMinPQ;
import java.awt.*;
import java.util.*;
import java.util.List;


public class AStarSeamCarver implements SeamCarver {
    private Picture picture;
    private Map<Vertex, Vertex> edgeTo = new HashMap<>();
    private Map<Vertex, Double> distTo = new HashMap<>();
    private boolean endIsMet = false;
    private Vertex start;
    private Vertex end;


    public AStarSeamCarver(Picture picture) {
        if (picture == null) {
            throw new NullPointerException("Picture cannot be null.");
        }
        this.picture = new Picture(picture);
    }

    public Picture picture() {
        return new Picture(picture);
    }

    public void setPicture(Picture picture) {
        this.picture = picture;
    }

    public int width() {
        return picture.width();
    }

    public int height() {
        return picture.height();
    }

    public Color get(int x, int y) {
        return picture.get(x, y);
    }

    public int[] findHorizontalSeam() {
        int[] horizontalSeam = new int[width()];
        if (height() == 1 && width() == 1) {
            return horizontalSeam;
        }
        if (height() == 1) {
            return horizontalSeam;
        }
        ExtrinsicMinPQ<Vertex> fringe = new TreeMapMinPQ<>(); //create a fringe
        this.start = new Vertex(-1, -1);
        edgeTo.put(start, null);
        distTo.put(start, 0.0);
        List<WeightedEdge<Vertex>> neighborEdges = new LinkedList<>();

        for (int i = 0; i < height(); i++) { //column is 0. row is i //x is column, y is row
            WeightedEdge<Vertex> e = new WeightedEdge<>(start, new Vertex(0, i), energy(0, i));
            neighborEdges.add(e);
        }
        addToFringe(neighborEdges, fringe);
        while (!fringe.isEmpty()) {
            Vertex next = fringe.removeSmallest();

            if (next.x == width() - 1) { //change
                end = next;
                endIsMet = true;
                break;
            }
            int colIndex = next.x + 1;
            neighborEdges = neighborsHorizontal(colIndex, next);
            addToFringe(neighborEdges, fringe);
        }
        shortestPathX(horizontalSeam);
        edgeTo = new HashMap<>();
        distTo = new HashMap<>();
        return horizontalSeam;
    }

    private void shortestPathX(int[] seam) {
        Vertex x = end;
        seam[0] = end.y;
        for (int i = 1; i < seam.length; i++) {
            x = edgeTo.get(x);
            seam[i] = x.y;
        }
        for (int i = 0; i < seam.length / 2; i++) {
            int temp = seam[i];
            seam[i] = seam[seam.length - i - 1];
            seam[seam.length - i - 1] = temp;
        }
    }

    private List<WeightedEdge<Vertex>> neighborsHorizontal(int colIndex, Vertex v) {
        List<WeightedEdge<Vertex>> neighborEdges = new LinkedList<>();
        if (colIndex > width() - 1) { // change v.x to colIndex. add = sign
            return neighborEdges;
        }
        if (v.y == 0) {
            WeightedEdge<Vertex> e1 = new WeightedEdge<>(v, new Vertex(colIndex, v.y), energy(colIndex, v.y));
            WeightedEdge<Vertex> e2 = new WeightedEdge<>(v, new Vertex(colIndex, v.y + 1), energy(colIndex, v.y + 1));
            neighborEdges.add(e1);
            neighborEdges.add(e2);
            return neighborEdges;
        }
        if (v.y == (height() - 1)) {
            WeightedEdge<Vertex> e1 = new WeightedEdge<>(v, new Vertex(colIndex, v.y), energy(colIndex, v.y));
            WeightedEdge<Vertex> e2 = new WeightedEdge<>(v, new Vertex(colIndex, v.y - 1), energy(colIndex, v.y - 1));
            neighborEdges.add(e1);
            neighborEdges.add(e2);
            return neighborEdges;
        }
        WeightedEdge<Vertex> e1 = new WeightedEdge<>(v, new Vertex(colIndex, v.y), energy(colIndex, v.y));
        WeightedEdge<Vertex> e2 = new WeightedEdge<>(v, new Vertex(colIndex, v.y + 1), energy(colIndex, v.y + 1));
        WeightedEdge<Vertex> e3 = new WeightedEdge<>(v, new Vertex(colIndex, v.y - 1), energy(colIndex, v.y - 1));
        neighborEdges.add(e1);
        neighborEdges.add(e2);
        neighborEdges.add(e3);
        return neighborEdges;
    }


    private void addToFringe(List<WeightedEdge<Vertex>> neighborEdges, ExtrinsicMinPQ<Vertex> fringe) {
        if (neighborEdges == null) {
            return;
        }
        for (WeightedEdge<Vertex> wev : neighborEdges) {
            Vertex s = wev.from(); //getting the starting vertex
            Vertex d = wev.to(); // getting the destination vertex
            double distance = wev.weight() + distTo.get(s);

            if (!distTo.containsKey(d)) {
                distTo.put(d, distance);
                edgeTo.put(d, s);
                fringe.add(d, distTo.get(d));
            } else if (distTo.get(d) > distance) { //relaxation
                distTo.replace(d, distance);
                edgeTo.replace(d, s);

                if (fringe.contains(d)) {
                    fringe.changePriority(d, distTo.get(d));
                } else {
                    fringe.add(d, distTo.get(d));
                }
            }
        }
    }

    public int[] findVerticalSeam() {
        int[] verticalSeam = new int[height()];

        if (height() == 1 && width() == 1) {
            return verticalSeam;
        }
        if (width() == 1) {
            return verticalSeam;
        }

        ExtrinsicMinPQ<Vertex> fringe = new TreeMapMinPQ<Vertex>(); //create a fringe
        this.start = new Vertex(-1, -1);
        edgeTo.put(start, null);
        distTo.put(start, 0.0);
        List<WeightedEdge<Vertex>> neighborEdges = new LinkedList<>();

        for (int i = 0; i < width(); i++) { //column is 0. row is i
            WeightedEdge<Vertex> e = new WeightedEdge<>(start, new Vertex(i, 0), energy(i, 0));
            neighborEdges.add(e);
        }

        addToFringe(neighborEdges, fringe);

        while (!fringe.isEmpty()) {
            Vertex next = fringe.removeSmallest();
            if (next.y == height() - 1) {
                end = next;
                endIsMet = true;
                break;
            }
            int rowIndex = next.y + 1;
            neighborEdges = neighborsVertical(rowIndex, next);
            addToFringe(neighborEdges, fringe);
        }
        shortestPathY(verticalSeam);
        edgeTo = new HashMap<>();
        distTo = new HashMap<>();
        return verticalSeam;
    }

    private List<WeightedEdge<Vertex>> neighborsVertical(int rowIndex, Vertex v) {
        List<WeightedEdge<Vertex>> neighborEdges = new LinkedList<>();
        if (rowIndex > height() - 1) { //error
            return new ArrayList<>();
        }
        if (v.x == 0) {
            WeightedEdge<Vertex> e1 = new WeightedEdge<>(v, new Vertex(0, rowIndex), energy(0, rowIndex));
            WeightedEdge<Vertex> e2 = new WeightedEdge<>(v, new Vertex(1, rowIndex), energy(1, rowIndex));
            neighborEdges.add(e1);
            neighborEdges.add(e2);
            return neighborEdges;
            //the weight is the energy of destination vertex
        }
        if (v.x == (width() - 1)) {
            WeightedEdge<Vertex> e1 = new WeightedEdge<>(v, new Vertex(v.x, rowIndex), energy(v.x, rowIndex));
            WeightedEdge<Vertex> e2 = new WeightedEdge<>(v, new Vertex(v.x - 1, rowIndex), energy(v.x - 1, rowIndex));
            neighborEdges.add(e1);
            neighborEdges.add(e2);
            return neighborEdges;
        }
        WeightedEdge<Vertex> e1 = new WeightedEdge<>(v, new Vertex(v.x, rowIndex), energy(v.x, rowIndex));
        WeightedEdge<Vertex> e2 = new WeightedEdge<>(v, new Vertex(v.x + 1, rowIndex), energy(v.x + 1, rowIndex));
        WeightedEdge<Vertex> e3 = new WeightedEdge<>(v, new Vertex(v.x - 1, rowIndex), energy(v.x - 1, rowIndex));
        neighborEdges.add(e1);
        neighborEdges.add(e2);
        neighborEdges.add(e3);
        return neighborEdges;
    }

    private void shortestPathY(int[] seam) {
        Vertex x = end;
        seam[0] = end.x;
        for (int i = 1; i < seam.length; i++) {
            x = edgeTo.get(x);
            seam[i] = x.x;
        }
        for (int i = 0; i < seam.length / 2; i++) {
            int temp = seam[i];
            seam[i] = seam[seam.length - i - 1];
            seam[seam.length - i - 1] = temp;
        }
    }

    private class Vertex extends Point {
        public Vertex(int x, int y) {
            this.x = x; //is the column
            this.y = y; //is the row
        }
        @Override
        public boolean equals(Object v) {
            if (!(v instanceof Vertex)) {
                return false;
            }
            return this.x == ((Vertex) v).x && this.y == ((Vertex) v).y;
        }

        @Override
        public int hashCode() {
            return super.hashCode();
        }
    }
}
