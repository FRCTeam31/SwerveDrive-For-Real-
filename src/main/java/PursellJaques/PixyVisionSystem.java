// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package PursellJaques;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.SPILink;
import io.github.pseudoresonance.pixy2api.links.UARTLink;


/** 
 * A class to use the pixy
 */
public class PixyVisionSystem {
    // Instance Variables
    Pixy2 pixy;

    /**
     * Create a new Pixy from the SPI link system
     */
    public PixyVisionSystem(){
        pixy = Pixy2.createInstance(new SPILink());
        System.out.println("PIXY CODE: " + pixy.init(5));
        pixy.setLED(255, 255, 255);
    }

    /**
     * 
     * @return the largets block from the pixy2 camera current image, null if none are found
     */
    public Block getBiggestBlock(int signature){
        int blockCount = pixy.getCCC().getBlocks(false, signature, 25);

        if(blockCount <= 0){
            // No blocks found
            return null;
        }
        ArrayList<Block> blocks = pixy.getCCC().getBlockCache(); // Get all the blocks from the pixy
        
        // Find block with largest area
        Block largestBlock = null;
        for(Block block: blocks){
            if(largestBlock == null){
                largestBlock = block;
            }
            if(block.getWidth() * block.getHeight() > largestBlock.getWidth() * largestBlock.getHeight()){
                largestBlock = block;
            }
        }
        return largestBlock;
    }

    /**
     * Display the x and y values of a block on smart dashboard
     * Will display <99999> for both if the block is null
     * @param block the block whose values will be displayed
     */
    public void displayBlock(Block block){
        if(block == null) {
            SmartDashboard.putNumber("Block X", 99999);
            SmartDashboard.putNumber("Block Y", 99999);
        }
        else{
            SmartDashboard.putNumber("Block X", block.getX());
            SmartDashboard.putNumber("Block Y", block.getY());
        }
        
    }

}
