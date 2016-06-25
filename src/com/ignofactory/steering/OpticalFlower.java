package com.ignofactory.steering;

import java.util.ArrayList;

import processing.core.*;
import processing.video.*;

// TODO eliminate edge artifacts (vectors point parallel to edge)
// make motion fit to torus

public class OpticalFlower {
	// Adapted from Optical Flow 2010/05/28
	// Hidetoshi Shimodaira shimo@is.titech.ac.jp 2010 GPL
	// from http://www.openprocessing.org/sketch/10435
	// Optical Flow by Hidetoshi Shimodaira, licensed under Creative Commons 
	// Attribution-Share Alike 3.0 and GNU GPL license.
	// Work: http://openprocessing.org/visuals/?visualID= 10435
	// License:
	// http://creativecommons.org/licenses/by-sa/3.0/
	// http://creativecommons.org/licenses/GPL/2.0/  

	// Adapted by Paul Hertz to run in Eclipse and in Processing
	// OpticalFlower 
	// To run in Eclipse (MacOS):
	//	    select Run Configurations from the Run menu
	//	    select your application from the list
	//	    select the Arguments tab 
	//	    enter -d32 in the VM arguments pane
	// Current versions of Eclipse use 64-bit architecture on machines that support it.
	// This fix enables 32-bit architecture allowing you to run 32-bit native code libraries.
	// See http://labs.makemachine.net/2010/02/processing-video-on-osx-10-6-with-eclipse/ for details.
	
	// This class is designed to track video in a buffer separate from the main window, so that
	// dimensions of the video buffer and optical flow grid do not have to conform to those of the main window.
	
	// Adapt as class called from a PApplet
	PApplet parent;
	VideoCallbackINF responder;

	int wscreen;                  // video screen width
	int hscreen;                  // video screen height
	int gs;                       // grid step (pixels)
	float predsec;                // prediction time (sec): larger for longer vector

	/*
	///////////////////////////////////////////////
	// typical parameters for desktop pc (high performance)
	int wscreen = 640;
	int hscreen = 480;
	int gs = 10;               // grid step (pixels)
	float predsec = 0.5f;      // prediction time (sec): larger for longer vector

	// typical parameters for laptop pc (low performance)
	//	int wscreen = 480;
	//	int hscreen = 360;
	//	int gs = 20;              // grid step (pixels)
	//	float predsec = 1.0f;      // prediction time (sec): larger for longer vector


	///////////////////////////////////////////////
	 */
	
	Capture video;
	PFont font;
	int[] vline;
	
	// flow line color
	int flowColor;
	int imageFlowColor;
	
	// flow line scale
	float flowScale = 1.0f;

	// capture parameters
	int fps = 30;
	String cameraName = "";

	// grid parameters

	int as;                         // window size for averaging (-as,...,+as)
	int gw;                         // number of horizontal grid spaces
	int gh;                         // number of vertical grid spaces
	int gs2;                        // half grid space
	float df;
	float vs;                       // vector scaling factor

	// regression vectors
	float[] fx, fy, ft;
	int fm = 3 * 9;                 // length of the vectors

	// regularization term for regression
	float fc = PApplet.pow(10, 8);          // larger values for noisy video

	// smoothing parameters
	float wflow = 0.1f;             // smaller value for longer smoothing

	// switches
	boolean flagseg = false;        // segmentation of moving objects?
	boolean flagmirror = true;      // mirroring image?
	boolean flagflow = true;        // draw opticalflow vectors?
	boolean flagimage = false;      // show video image ?

	// internally used variables
	float ar, ag, ab;               // used as return value of pixave
	float[] dtr, dtg, dtb;          // differentiation by t (red, green, blue)
	float[] dxr, dxg, dxb;          // differentiation by x (red, green, blue)
	float[] dyr, dyg, dyb;          // differentiation by y (red, green, blue)
	float[] par, pag, pab;          // averaged grid values (red, green, blue)
	float[] flowx, flowy;           // computed optical flow
	float[] sflowx, sflowy;                 // slowly changing version of the flow
	int clockNow, clockPrev, clockDiff;     // for timing check
	private PVector[] flowList;
	boolean isFirstSweep = true;

	int[] xcoords;
	int[] ycoords;
	float[] vxcoords;
	float[] vycoords;
	
	// Constructor requires camera name and other values
	/**
	 * @param parent        the PApplet instance
	 * @param wscreen       width of video
	 * @param hscreen       height of video
	 * @param fps           frames per second, 30 is recommended
	 * @param gs            number of pixels in a grid tile
	 * @param predsec       prediction interval
	 * @param cameraName    the camera to use
	 */
	public OpticalFlower(PApplet parent, int wscreen, int hscreen, int fps, int gs, float predsec, String cameraName) {
		this.parent = parent;
		this.wscreen = wscreen;
		this.hscreen = hscreen;
		this.fps = fps;
		this.gs = gs;
		this.predsec = predsec;
		this.cameraName = cameraName;
		flowColor = parent.color(220, 220, 220);
		imageFlowColor = parent.color(240, 240, 240, 64);
		init();
	}

	public void init(){
		// set up video capture
		PApplet.println("init: ", parent, wscreen, hscreen, cameraName, fps);
		video = new Capture(parent, wscreen, hscreen, cameraName, fps);
		video.start();
		// font
		font = parent.createFont("Verdana", 10);
		parent.textFont(font);
		// draw
		parent.rectMode(PApplet.CENTER);
		parent.ellipseMode(PApplet.CENTER);
		/**/
		// scaling (TODO release from hard coding)
		as = gs * 2;                // window size for averaging (-as,...,+as)
		gw = wscreen/gs;            // number of horizontal grid spaces
		gh = hscreen/gs;            // number of vertical grid spaces
		gs2 = gs/2;                 // half grid space
		df = predsec * fps;
		vs = parent.width / (float)wscreen;
		flowScale *= vs;
		PApplet.println("vs = "+ vs);
		// arrays
		par = new float[gw * gh];
		pag = new float[gw * gh];
		pab = new float[gw * gh];
		dtr = new float[gw * gh];
		dtg = new float[gw * gh];
		dtb = new float[gw * gh];
		dxr = new float[gw * gh];
		dxg = new float[gw * gh];
		dxb = new float[gw * gh];
		dyr = new float[gw * gh];
		dyg = new float[gw * gh];
		dyb = new float[gw * gh];
		flowx = new float[gw * gh];
		flowy = new float[gw * gh];
		sflowx = new float[gw * gh];
		sflowy = new float[gw * gh];
		fx = new float[fm];
		fy = new float[fm];
		ft = new float[fm];
		vline = new int[wscreen];        // color array
		flowList = new PVector[gw * gh]; // flow vectors
		loadPoints();
	}
	
	public int getGs() {
		return this.gs;
	}
	public int getGw() {
		return this.gw;
	}
	public int getGh() {
		return this.gh;
	}


	// calculate average pixel value (r, g, b) for rectangle region
	// called by first sweep in flow method
	void pixave(int x1, int y1, int x2, int y2) {
		float sumr, sumg, sumb;
		int pix;                        // a color
		int r, g, b;
		int n;
		// clip boundary values to bounds, if necessary
		if(x1 < 0) x1 = 0;
		if(x2 >= wscreen) x2 = wscreen - 1;
		if(y1 < 0) y1 = 0;
		if(y2 >= hscreen) y2 = hscreen - 1;
		// set color channel sums to 0
		sumr = sumg = sumb = 0.0f;
		for(int y = y1; y <= y2; y++) {
			for(int i = wscreen * y + x1; i <= wscreen * y + x2; i++) {
				pix = video.pixels[i];
				b = pix & 0xFF;       // blue
				pix = pix >> 8;
			    g = pix & 0xFF;       // green
			    pix = pix >> 8;
			    r = pix & 0xFF;       // red
			    // averaging the values
			    sumr += r;
			    sumg += g;
			    sumb += b;
			}
		}
		n = (x2 - x1 + 1) * (y2 - y1 + 1);      // number of pixels
		// store the results in ar, ag, ab for further calculation in first sweep in flow method
		ar = sumr/n;
		ag = sumg/n;
		ab = sumb/n;
	}

	// extract values from 9 neighbour grids
	void getnext9(float x[], float y[], int i, int j) {
		y[j + 0] = x[i + 0];
		y[j + 1] = x[i - 1];
		y[j + 2] = x[i + 1];
		y[j + 3] = x[i - gw];
		y[j + 4] = x[i + gw];
		y[j + 5] = x[i - gw - 1];
		y[j + 6] = x[i - gw + 1];
		y[j + 7] = x[i + gw - 1];
		y[j + 8] = x[i + gw + 1];
	}

	// solve optical flow by least squares (regression analysis)
	void solveflow(int ig) {
		float xx, xy, yy, xt, yt;
		float a, u, v, w;

		// prepare covariances
		xx = xy = yy = xt = yt = 0.0f;
		for(int i = 0; i < fm; i++) {
			xx += fx[i] * fx[i];
			xy += fx[i] * fy[i];
			yy += fy[i] * fy[i];
			xt += fx[i] * ft[i];
			yt += fy[i] * ft[i];
		}

		// least squares computation
		a = xx * yy - xy * xy + fc;       // fc is for stable computation
		u = yy * xt - xy * yt;            // x direction
		v = xx * yt - xy * xt;            // y direction

		// write back
		flowx[ig] = -2 * gs * u/a;        // optical flow x (pixel per frame)
		flowy[ig] = -2 * gs * v/a;        // optical flow y (pixel per frame)
	}
	
	void loadPoints() {
		xcoords = new int[gw * gh];
		ycoords = new int[gw * gh];
		vxcoords = new float[gw * gh];
		vycoords = new float[gw * gh];
		for(int ix = 0; ix < gw; ix++) {
			int x0 = ix * gs + gs2;
			for(int iy = 0; iy < gh; iy++) {
				int y0 = iy * gs + gs2;
				int ig = iy * gw + ix;
				xcoords[ig] = x0;
				ycoords[ig] = y0;
				vxcoords[ig] = x0 * vs;
				vycoords[ig] = y0 * vs;
			}
		}
	}
	
	// TODO optimize x0, y0, and ig calculations with array storage, 
	// since they never change after the first time through
	// calculate 2D vector array, draw it over parent display on request
	// also call callback method to display video on request
	public void flow() {
		if(video.available()){
			// video capture
			video.read();
			// clock in msec
			clockNow = parent.millis();
			clockDiff = clockNow - clockPrev;
			clockPrev = clockNow;

			// mirror, before any other calculations
			if(flagmirror) {
				for(int y = 0; y < hscreen; y++) {
					int ig = y * wscreen;
					for(int x = 0; x < wscreen; x++)
						vline[x] = video.pixels[ig + x];
					for(int x = 0; x < wscreen; x++)
						video.pixels[ig + x] = vline[wscreen - 1 - x];
				}
			}

			// draw image
			if(flagimage) {
				if (null != responder) {
					responder.videoCallback(video);
				}
				else {
					if (video.available()) (parent).set(0, 0, video);
				}
			}

			// 1st sweep: differentiation by time
			for(int ix = 0; ix < gw; ix++) {
				int x0 = ix * gs + gs2;
				for(int iy = 0; iy < gh; iy++) {
					int ig = iy * gw + ix;
					// int y0 = iy * gs + gs2;
					int y0 = ycoords[ig];
					// compute average pixel at (x0, y0)
					pixave(x0 - as, y0 - as, x0 + as, y0 + as);
					// compute time difference
					dtr[ig] = ar - par[ig];         // red
					dtg[ig] = ag - pag[ig];         // green
					dtb[ig] = ab - pab[ig];         // blue
					// save the pixel
					par[ig]=ar;
					pag[ig]=ag;
					pab[ig]=ab;
				}
			}

			// 2nd sweep: differentiations by x and y
			for(int ix = 1; ix < gw - 1; ix++) {
				for(int iy = 1; iy < gh - 1; iy++) {
					int ig = iy * gw + ix;
					// compute x difference
					dxr[ig] = par[ig + 1]-par[ig - 1];      // red
					dxg[ig] = pag[ig + 1]-pag[ig - 1];      // green
					dxb[ig] = pab[ig + 1]-pab[ig - 1];      // blue
					// compute y difference
					dyr[ig] = par[ig + gw]-par[ig - gw];    // red
					dyg[ig] = pag[ig + gw]-pag[ig - gw];    // green
					dyb[ig] = pab[ig + gw]-pab[ig - gw];    // blue
				}
			}

			// 3rd sweep: solving optical flow
			for(int ix = 1; ix < gw - 1; ix++) {
				for(int iy = 1; iy < gh - 1; iy++) {
					int ig = iy * gw + ix;
					
					// prepare vectors fx, fy, ft
					getnext9(dxr, fx, ig, 0);           // dx red
					getnext9(dxg, fx, ig, 9);           // dx green
					getnext9(dxb, fx, ig, 18);          // dx blue
					getnext9(dyr, fy, ig, 0);           // dy red
					getnext9(dyg, fy, ig, 9);           // dy green
					getnext9(dyb, fy, ig, 18);          // dy blue
					getnext9(dtr, ft, ig, 0);           // dt red
					getnext9(dtg, ft, ig, 9);           // dt green
					getnext9(dtb, ft, ig, 18);          // dt blue

					// solve for (flowx, flowy) such that
					// fx flowx + fy flowy + ft = 0
					solveflow(ig);

					// smoothing
					sflowx[ig] += (flowx[ig] - sflowx[ig]) * wflow;
					sflowy[ig] += (flowy[ig] - sflowy[ig]) * wflow;
				}
			}

			// 4th sweep: calculate and store the vectors
			if(flagflow) {
				for(int iy = 0; iy < gh; iy++) {
					for(int ix = 0; ix < gw; ix++) {
						int ig = iy * gw + ix;
						float u = df * sflowx[ig];
						float v = df * sflowy[ig];
						// save the vectors
						flowList[ig] = new PVector(u, v);
						//if (isFirstSweep) PApplet.print(ig + "  ");
					}
					//if (isFirstSweep) PApplet.println();
				}
				// callback handles drawing
				responder.vectorCallback(video);
			}
			isFirstSweep = false;
			
			if (null != responder) responder.actionCallback(video);
		}
	}
	
	/**
	 * @param flowColor the flowColor to set
	 */
	public void setFlowColor(int flowColor) {
		this.flowColor = flowColor;
	}

	/**
	 * @param imageFlowColor the imageFlowColor to set
	 */
	public void setImageFlowColor(int imageFlowColor) {
		this.imageFlowColor = imageFlowColor;
	}

	/**
	 * @return the flowScale
	 */
	public float getFlowScale() {
		return flowScale;
	}

	/**
	 * @param flowScale the flowScale to set
	 */
	public void setFlowScale(float flowScale) {
		this.flowScale = flowScale;
	}
	
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
	 * various methods to return vector at specified position
	 * where possible, we get the vector from array flowList rather than calculating it
	 * 
	 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	public int gridIndex(int x, int y) {
		// find the grid index from display coordinates x and y
		int ix = PApplet.round((x/vs - gs2)/gs);
		int iy = PApplet.round((y/vs - gs2)/gs);
		// PApplet.println("x = "+ x +", y = "+ y +", ix = "+ ix +", iy = "+ iy);
		if(ix < 1) ix = 1;
		else if(ix > gw - 2) ix = gw - 2;
		if(iy < 1) iy = 1;
		else if(iy > gh - 2) iy = gh - 2;
		return iy * gw + ix;
	}
	
	
	/**
	 * Returns vector at position in the video corresponding to display coordinates x, y
	 * @param x
	 * @param y
	 * @return
	 */
	public PVector getFlow(int x, int y) {
		// find the grid
		int ig = gridIndex(x, y);
		return getGridFlow(ig, 0.3f);
	}
	
	public PVector getGridFlow(int ig, float trim) {
		// first call may arrive before flowList is initialized
		if (null == flowList[ig]) {
			float u = sflowx[ig];
			float v = sflowy[ig];
			float a = PApplet.sqrt(u * u + v * v);
			if (a < trim) return new PVector(0,0);
			PApplet.println("calculated vector");
			return new PVector(u/a, v/a);
		}
		else {
			return flowList[ig];
		}
	}
	
	/**
	 * returns vector at supplied position (PVector) in the video
	 * @param  vec a PVector indicating the position of a point in the display window
 	 * @return flow vector at integer coordinate derived from supplied vector
	 */
	public PVector getFlow(PVector vec) {
		return getFlow(Math.round(vec.x), Math.round(vec.y));
	}
	
	/**
	 * @return the xcoords, coordinates in the grid
	 */
	public int[] getXcoords() {
		return xcoords;
	}

	/**
	 * @return the ycoords, coordinates in the grid
	 */
	public int[] getYcoords() {
		return ycoords;
	}

	/**
	 * @return the vxcoords, coordinates in the display
	 */
	public float[] getVxcoords() {
		return vxcoords;
	}

	/**
	 * @return the vycoords, coordinates in the display 
	 */
	public float[] getVycoords() {
		return vycoords;
	}

	public void toggleFlowDisplay() {
		flagflow = !flagflow;
		PApplet.println("flagflow = "+ flagflow);
	}	
	/**
	 * @return the flagflow
	 */
	public boolean isFlagflow() {
		return flagflow;
	}

	public PVector[] getFlowList() {
		return flowList;
	}

	public void toggleImageDisplay() {
		flagimage = !flagimage;
		PApplet.println("flagimage = "+ flagimage);
	}
	public void showImage() {
		flagimage = true;
	}
	public void hideImage() {
		flagimage = false;
	}
	/**
	 * @return the flagimage
	 */
	public boolean isFlagimage() {
		return flagimage;
	}

	/**
	 * @return the responder
	 */
	public VideoCallbackINF getResponder() {
		return responder;
	}

	/**
	 * @param responder the responder to set
	 */
	public void setResponder(VideoCallbackINF responder) {
		this.responder = responder;
	}

	public Capture getVideo() {
		return this.video;
	}
	
}
