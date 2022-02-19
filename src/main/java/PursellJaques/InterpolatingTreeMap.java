package PursellJaques;

import java.util.TreeMap;

/**
 * An interpolating tree map (can return a value of a non-existent key based on other key-value pairs)
 */
public class InterpolatingTreeMap {
    public TreeMap<Double, Double> treeMap;
    private double maxKey;
    private double minKey;

    /**
     * Enter in first key-value pair
     * @param key first key
     * @param value first value
     */
    public InterpolatingTreeMap(double key, double value){
        treeMap = new TreeMap<Double, Double>();
        treeMap.put(key, value);
        maxKey = key;
        minKey = key;
    }

    /**
     * Add a new key-value pair
     * @param key
     * @param value
     */
    public void put(double key, double value){
        treeMap.put(key, value);
        if(key > maxKey){
            maxKey = key;
        }
        if(key < minKey){
            minKey = key;
        }
    }

    /**
     * Return the lowest key that is higher than the inpout key, or the max key if none exist
     */
    public double getKeyCeiling(double key){
        if(key >= maxKey){
            return maxKey;
        }
        else{
            return (double) treeMap.ceilingKey(key);
        }
    }

    /**
     * Return the highest key that is less than the input key, or the min key if none exist
     */
    public double getKeyFloor(double key){
        if(key <= minKey){
            return minKey;
        }
        else{
            return (double) treeMap.floorKey(key);
        }
    }

    /**
     * Return the value of a key. If the key does not exist, return the value that is greated from the linear interpolation from the two nearest key-value pairs 
     * where one key is less than the input, and another is greater than the iput. If the input key is larger or smaller
     * than the max or min recorded keys, return the value of the max or min key
     * @param key
     * @return
     */
    public double getInterpolatedKey(double key){
        if(key >= maxKey){
            return (double) treeMap.get(maxKey);
        }
        if(key <= minKey){
            return (double) treeMap.get(minKey);
        }
        double keyFloor = this.getKeyFloor(key);
        double keyCeiling = this.getKeyCeiling(key);

        double keyDifference = keyCeiling - keyFloor;

        return (keyCeiling - key) / keyDifference * ((double) treeMap.get(keyFloor)) + (key - keyFloor) / keyDifference * ((double) treeMap.get(keyCeiling));
    }
}