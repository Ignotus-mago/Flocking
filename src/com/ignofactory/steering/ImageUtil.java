package com.ignofactory.steering;

import processing.core.*;

public class ImageUtil {
	PApplet parent;
	
	/**
	 * @param parent
	 */
	public ImageUtil(PApplet parent) {
		super();
		this.parent = parent;
	}

	/**
	 * Copy an image, work around problems in Processing 2.1 with PImage.get.
	 * Faster than get() anyway.
	 * @param image   image to copy
	 * @return        a copy of the image submitted
	 */
	public PImage copyImagePixels(PImage image) {
		int h = image.height;
		int w = image.width;
		PImage newImage = parent.createImage(w, h, PImage.ARGB);
		newImage.loadPixels();
		PApplet.arrayCopy(image.pixels, newImage.pixels);
		newImage.updatePixels();
		return newImage;
	}
	
	/**
	 * Copy an image pixel by pixel, test code to work around problems in Processing 2.1 with PImage.get.
	 * @param image   image to copy
	 * @return        the image submitted with apha channel set to desired value
	 */
	public PImage loadImageAlpha(PImage image, int alpha) {
		int i = 0;
		image.loadPixels();
		for (i = 0; i < image.pixels.length; i++) {
			int[] rgb = rgbComponents(image.pixels[i]);
			image.pixels[i] = alpha << 24 | rgb[0] << 16 | rgb[1] << 8 | rgb[2];
		}
		image.updatePixels();
		return image;
	}
	
	/**
	 * Breaks a Processing color into R, G and B values in an array.
	 * @param argb   a Processing color as a 32-bit integer 
	 * @return       an array of integers in the intRange 0..255 for 3 primary color components: {R, G, B}
	 */
	public static int[] rgbComponents(int argb) {
		int[] comp = new int[3];
		comp[0] = (argb >> 16) & 0xFF;  // Faster way of getting red(argb)
		comp[1] = (argb >> 8) & 0xFF;   // Faster way of getting green(argb)
		comp[2] = argb & 0xFF;          // Faster way of getting blue(argb)
		return comp;
	}

	/**
	 * Returns alpha channel value of a color.
	 * @param argb   a Processing color as a 32-bit integer 
	 * @return       an int for alpha channel
	 */
	public static int alphaComponent(int argb) {
		return (argb >> 24);
	}

	/**
	 * Creates a Processing ARGB color from r, g, b, and alpha channel values. Note the order
	 * of arguments, the same as the Processing color(value1, value2, value3, alpha) method. 
	 * @param r   red component 0..255
	 * @param g   green component 0..255
	 * @param b   blue component 0..255
	 * @param a   alpha component 0..255
	 * @return    a 32-bit integer with bytes in Processing format ARGB.
	 */
	public static int composeColor(int r, int g, int b, int a) {
		return a << 24 | r << 16 | g << 8 | b;
	}

	/**
	 * Creates a Processing ARGB color from r, g, b, values in an array. 
	 * @param comp   array of 3 integers in range 0..255, for red, green and blue components of color
	 *               alpha value is assumed to be 255
	 * @return       a 32-bit integer with bytes in Processing format ARGB.
	 */
	public static int composeRGBColor(int[] comp) {
		return 255 << 24 | comp[0] << 16 | comp[1] << 8 | comp[2];
	}
	
	/**
	 * Creates a Processing ARGB color from r, g, b, a values in an array. 
	 * @param comp   array of 4 integers in range 0..255, for red, green, blue and alpha components of color
	 * @return       a 32-bit integer with bytes in Processing format ARGB.
	 */
	public static int composeRGBAColor(int[] comp) {
		return comp[3] << 24 | comp[0] << 16 | comp[1] << 8 | comp[2];
	}


}
