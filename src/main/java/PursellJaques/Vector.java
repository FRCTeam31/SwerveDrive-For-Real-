package PursellJaques;

/**
 * This class is a tool for instantiating and performing mathematical operations on vectors.
 * @authors Miles Jaques, Noah Pursell
 */
public class Vector {

    private Double[] components;
    private int size;
    private double magnitude;

    /**
     * Creates a vector with the specified components
     * @param components The components of the vector.
     */
    public Vector(Double[] components) {
        this.components = components;
        size = components.length;
        double acc = 0;
        for (double d: components) {
            acc += d*d;
        }
        magnitude = Math.sqrt(acc);
    }

      /**
     * Creates a vector with the specified components
     * @param x The x-component
     * @param y The y-component
     */
    public Vector(double x, double y) {
        this.components = new Double[] {x, y};
        size = 2;
        magnitude = Math.sqrt(x*x + y*y);
    }

    /**
     * Creates a zero vector with specified dimensions.
     * @param size The number of dimensions or the size of the vector.
     */
    public Vector(int size) {
        components = new Double[size];
        for (int i = 0; i < size; i++) {
            components[i] = 0.0;
        }
        this.size = size;
        magnitude = 0;
    }

    public String toString() {
        String s = "<"+components[0]+", ";
        for (int i = 1; i < size-1; i++) {
            s += components[i] + ", ";
        }
        return s + components[size-1] + ">";
    }

    /**
     * 
     * @return the components array of the vector
     */
    public Double[] getComponents() {return components;}
    /**
     * 
     * @return the number of elements in the vector's components array
     */
    public int getSize() {return size;}

    /**
     * Adds two vectors.
     * @param v the vector to be added
     * @return A vector that is th the addition of two same-size vectors.
     */
    public Vector add(Vector v) throws ArithmeticException {
        Double[] otherComponents = v.getComponents();
        if (size != otherComponents.length) {
            throw new ArithmeticException("can only operate on same dimensional vectors");
        }
        Double[] newComponents = new Double[size];

        for (int i = 0; i < size; i++) {
            newComponents[i] = components[i] + otherComponents[i];
        }

        return new Vector(newComponents);
    }

    /**
     * Adds two vectors.
     * @param v the vector to be subtracted.
     * @return A vector that is the difference of two same-size vectors.
     * @throws ArithmeticException
     */
    public Vector subtract(Vector v) throws ArithmeticException {
        Double[] otherComponents = v.getComponents();
        if (size != otherComponents.length) {
            throw new ArithmeticException("can only operate on same dimensional vectors");
        }
        Double[] newComponents = new Double[size];

        for (int i = 0; i < size; i++) {
            newComponents[i] = components[i] - otherComponents[i];
        }

        return new Vector(newComponents);
    }

    /**
     * 
     * @param v the other vector
     * @return  a scalar 
     * @throws ArithmeticException
     */
    public double dotProduct(Vector v) throws ArithmeticException {
        Double[] otherComponents = v.getComponents();
        if (size != otherComponents.length) {
            throw new ArithmeticException("can only operate on same dimensional vectors");
        }
        double product = 0;

        for (int i = 0; i < size; i++) {
            product += components[i] * otherComponents[i];
        }

        return product;
    }

    /**
     * 
     * @param v the other vector
     * @return a vector that is perpendicular to both {@code v} and {@code this}
     * @throws IllegalArgumentException
     */
    public Vector crossProduct(Vector v) throws IllegalArgumentException{
        Double[] otherComponents = v.getComponents();
        if (!(size == 3 && otherComponents.length == 3)) {
            throw new IllegalArgumentException("cannot operate on non 3-dimensional vectors");
        }

        return new Vector(new Double[] {components[1]*otherComponents[2]-components[2]*otherComponents[1],
                                        (components[0]*otherComponents[2]-components[2]*otherComponents[0])*-1,
                                        components[0]*otherComponents[1]-components[1]*otherComponents[0]});
    }

    /**
     * 
     * @return a vector that is perpendicular in the clockwise direction
     * @throws Exception
     */
    public Vector getPerpendicular() throws Exception {
        if(this.size != 2){
            throw new Exception("Vector must be 2-dimensional");
        }

        return new Vector(components[1], -1 * components[0]);
    }

    /**
     * 
     * @param scalar the scalar to multiply the vector by
     * @return a scaled vector
     */
    public Vector scalarMultiplication(double scalar) {
        Double[] newComponents = new Double[size];
        for (int i = 0; i < size; i++) {
            newComponents[i] = components[i]*scalar;
        }

        return new Vector(newComponents);
    }

    /**
     * 
     * @return the angle of a 2D vector relative to the normal line
     */
    public double getTheta() {
        if(components[0] == 0 && components[1] == 0){
            return 1;
        }
        return Math.toDegrees(Math.atan2(components[0], components[1]));
    }

    /**
     * 
     * @return the magnitude of the vector
     */
    public double getMagnitude() {return magnitude;}
}