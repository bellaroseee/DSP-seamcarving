package seamcarving;

import edu.princeton.cs.algs4.Picture;

import java.awt.Color;

public interface SeamCarver {

    /** Returns the current image. (This updates whenever a seam is removed.) */
    Picture picture();

    /** Sets the current image. */
    void setPicture(Picture picture);

    /** Returns the width of the current image, in pixels. */
    int width();

    /** Returns the height of the current image, in pixels. */
    int height();

    /** Returns the color of pixel (x, y) in the current image. */
    Color get(int x, int y);



    /** Returns the energy of pixel (x, y) in the current image. */
    default double energy(int x, int y) {

        if (height() == 1 && width() == 1) {
            return 0.0;
        }

        if (height() == 1) {
            return energyHelper(x, y);
        }
        if (width() == 1) {
            return energyHelperW(x, y);
        }
        double gradientSqX = 0;
        double gradientSqY = 0;

        if (x == 0) {
            if (y == 0) {
                gradientSqX = gradientSq(get(1, 0), get(width() - 1, 0));
                gradientSqY = gradientSq(get(0, 1), get(0, height() - 1));
                return Math.sqrt(gradientSqX + gradientSqY);
            }
            if (y > 0 && y < height() - 1) {
                gradientSqX = gradientSq(get(1, y), get(width() - 1, y));
                gradientSqY = gradientSq(get(x, y + 1), get(x, y - 1));
                return Math.sqrt(gradientSqX + gradientSqY);
            }
            if (y == height() - 1) {
                gradientSqX = gradientSq(get(1, height() - 1), get(width() - 1, height() - 1));
                gradientSqY = gradientSq(get(0, 0), get(0, height() - 2));
                return Math.sqrt(gradientSqX + gradientSqY);
            }
        }
        if (x == width() - 1) {
            if (y == 0) {
                gradientSqX = gradientSq(get(width() - 2, 0), get(0, 0));
                gradientSqY = gradientSq(get(width() - 1, 1), get(width() - 1, height() - 1));
                return Math.sqrt(gradientSqX + gradientSqY);
            }
            if (y > 0 && y < height() - 1) {
                gradientSqX = gradientSq(get(width() - 2, y), get(0, y));
                gradientSqY = gradientSq(get(x, y + 1), get(x, y - 1));
                return Math.sqrt(gradientSqX + gradientSqY);
            }
            if (y == height() - 1) {
                gradientSqX = gradientSq(get(width() - 2, height() - 1), get(0, height() - 1));
                gradientSqY = gradientSq(get(width() - 1, height() - 2), get(width() - 1, 0));
                return Math.sqrt(gradientSqX + gradientSqY);
            }
        }
        if (x > 0 && x < width() - 1) {
            if (y == 0) {
                gradientSqX = gradientSq(get(x + 1, y), get(x - 1, y));
                gradientSqY = gradientSq(get(x, y + 1), get(x, height() - 1));
                return Math.sqrt(gradientSqX + gradientSqY);
            }
            if (y == height() - 1) {
                gradientSqX = gradientSq(get(x + 1, y), get(x - 1, y));
                gradientSqY = gradientSq(get(x, 0), get(x, height() - 2));
                return Math.sqrt(gradientSqX + gradientSqY);
            }
        }
        //for pixels that are on the center part of image.
        gradientSqX = gradientSq(get(x + 1, y), get(x - 1, y));
        gradientSqY = gradientSq(get(x, y + 1), get(x, y - 1));

        return Math.sqrt(gradientSqX + gradientSqY);
    }

    private double energyHelper(int x, int y) {
        if (x == 0) {
            return Math.sqrt(gradientSq(get(1, 0), get(width() - 1, 0)));
        }
        if (x == width() - 1) {
            return Math.sqrt(gradientSq(get(width() - 2, 0), get(0, 0)));
        }
        return Math.sqrt(gradientSq(get(x + 1, y), get(x - 1, y)));
    }

    private double energyHelperW(int x, int y) {
        if (y == 0) {
            return Math.sqrt(gradientSq(get(0, 1), get(0, height() - 1)));
        }
        if (y == height() - 1) {
            return Math.sqrt(gradientSq(get(0, 0), get(0, height() - 2)));
        }
        return Math.sqrt(gradientSq(get(x, y + 1), get(x, y - 1)));
    }

    private double gradientSq(Color c1, Color c2) {
        int red = Math.abs(c1.getRed() - c2.getRed());
        int green = Math.abs(c1.getGreen() - c2.getGreen());
        int blue = Math.abs(c1.getBlue() - c2.getBlue());

        double gradientSq = Math.pow(red, 2) + Math.pow(green, 2) + Math.pow(blue, 2);
        return gradientSq;
    }

    /** Returns true iff pixel (x, y) is in the current image. */
    default boolean inBounds(int x, int y) {
        return (x >= 0) && (x < width()) && (y >= 0) && (y < height());
    }

    /**
     * Calculates and returns a minimum-energy horizontal seam in the current image.
     * The returned array will have the same length as the width of the image.
     * A value of v at index i of the output indicates that pixel (i, v) is in the seam.
     */
    int[] findHorizontalSeam();

    /**
     * Calculates and returns a minimum-energy vertical seam in the current image.
     * The returned array will have the same length as the height of the image.
     * A value of v at index i of the output indicates that pixel (v, i) is in the seam.
     */
    int[] findVerticalSeam();

    /** Calculates and removes a minimum-energy horizontal seam from the current image. */
    default void removeHorizontalSeam(int[] seam) {
        if (seam == null) {
            throw new NullPointerException("Input seam array cannot be null.");
        } else if (width() == 1) {
            throw new IllegalArgumentException("Image width is 1.");
        } else if (seam.length != width()) {
            throw new IllegalArgumentException("Seam length does not match image width.");
        }

        for (int i = 0; i < seam.length - 2; i++) {
            if (Math.abs(seam[i] - seam[i + 1]) > 1) {
                throw new IllegalArgumentException(
                        "Invalid seam, consecutive vertical indices are greater than one apart.");
            }
        }

        Picture carvedPicture = new Picture(width(), height() - 1);
        /* Copy over the all indices besides the index specified by the seam */
        for (int i = 0; i < width(); i++) {
            for (int j = 0; j < seam[i]; j++) {
                carvedPicture.set(i, j, get(i, j));
            }

            for (int j = seam[i] + 1; j < height(); j++) {
                carvedPicture.set(i, j - 1, get(i, j));
            }
        }

        setPicture(carvedPicture);
    }

    /** Calculates and removes a minimum-energy vertical seam from the current image. */
    default void removeVerticalSeam(int[] seam) {
        if (seam == null) {
            throw new NullPointerException("Input seam array cannot be null.");
        } else if (height() == 1) {
            throw new IllegalArgumentException("Image height is 1.");
        } else if (seam.length != height()) {
            throw new IllegalArgumentException("Seam length does not match image height.");
        }

        for (int i = 0; i < seam.length - 2; i++) {
            if (Math.abs(seam[i] - seam[i + 1]) > 1) {
                throw new IllegalArgumentException(
                        "Invalid seam, consecutive horizontal indices are greater than one apart.");
            }
        }

        Picture carvedPicture = new Picture(width() - 1, height());
        for (int i = 0; i < height(); i++) {
            for (int j = 0; j < seam[i]; j++) {
                carvedPicture.set(j, i, get(j, i));
            }

            for (int j = seam[i] + 1; j < width(); j++) {
                carvedPicture.set(j - 1, i, get(j, i));
            }
        }

        setPicture(carvedPicture);
    }
}
