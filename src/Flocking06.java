import java.awt.Container;
import java.awt.Frame;
import java.awt.Toolkit;
import java.awt.event.KeyEvent;
import java.awt.geom.Point2D;
import java.io.PrintWriter;
import java.util.ArrayList;
import net.paulhertz.geom.GeomUtils;
import net.paulhertz.geom.Matrix3;
import net.paulhertz.util.RandUtil;
import com.ignofactory.steering.*;
import net.paulhertz.aifile.*;
import controlP5.*;
import processing.core.*;
import processing.video.*;


/**
 * @author paulhz
 * Licensed under the GNU General Public License version 3.0, which should accompany any distribution of this software.
 * 
 * Using Processing 2 in Eclipse, but this will change to 3.x. Files in ProcessingDemo folder in this project
 * will run under Processing 3.x. 
 * 
 * Seventh version of flocking application, for gallery installation. 
 * Based on Flocking, by Daniel Shiffman <http://www.shiffman.net>, in The Nature of Code, Spring 2009.
 * Demonstration of Craig Reynolds' "Flocking" behavior, See: http://www.red3d.com/cwr/.
 * Rules: Cohesion, Separation, Alignment
 * 
 * Requires IgnoCodeLib (see https://processing.org/reference/libraries/) and ControlP5 (http://www.sojamo.de/libraries/controlP5/).
 * Runs in the Eclipse IDE, but can be modified to run in Processing. 
 * 
 * Implements video tracking. You will probably need to tweak video for your particular setup. 
 * The call "setupVideo(videoWidth, videoHeight, 30, 8, 0.25f, "FaceTime HD Camera");" in setup 
 * will have to be changed for your particular camera and screen dimensions. See the setupVideo method
 * for some examples of different calls. Width and height of your display window may need to be adjusted, too.
 * A call to printDevices() can tell you the video devices you have available.
 * 
 * I added some file naming methods that pretty much assure no two files get the same name.
 * Basically, files are time-stamped to the second.
 * 
 * This is my development version as of 2016. Forgive the spaghetti code, I'm an artist. ;^}.
 * 
 */
public class Flocking06 extends PApplet {
	// TODO find all the places where torus-connectivity needs to be used.
	// Written by Paul Hertz
	// Requires Processing library IgnoCodeLib 0.2b6 and above
	// available at http://paulhertz.net/ignocodelib/
	// Requires ControlP5 library
	// available at http://www.sojamo.de/libraries/controlP5/

	// Based on Flocking
	// by Daniel Shiffman <http://www.shiffman.net>
	// The Nature of Code, Spring 2009

	// Demonstration of Craig Reynolds' "Flocking" behavior
	// See: http://www.red3d.com/cwr/
	// Rules: Cohesion, Separation, Alignment

	// Boids also have mass that determines effects of external forces (i.e., wind)
	
	// TODO first "wraparound" point on torus should be mapped from last point. This will make patterns simpler to create.


	// SHORTCUTS
	//	  Press spacebar to show or hide controls
	//	  Press 'd' to toggle drawing
	//  	Press 'r' to toggle immediate display of drawing
	//  	Press 'c' to shift color
	//  	Press 'x' to erase
	//		Press 'p' to pause
	//		Press 's' to save to AI 7.0 format file
	//		Press 'v' to toggle visibility of boids
	//		Press 'l' to toggle cohesion lines (best with few boids)
	//		Press 't' to toggle topology from torus to plane
	//		Press 'w' to toggle wind
	//		Press '+' or '-' to increase or decrease boids spearation
	//		Press UP or DOWN arrow keys to increase or decrease wind force
	//		Press LEFT or RIGHT arrow keys to turn boids.
	//		Press mouse to attract boids (when controls are hidden)
	//		Drag mouse to push or pull boids (when controls are hidden)
	//		Press 'g' to toggle stop drawing at edge
	//		Press 'b' to toggle start drawing at edge
	//		Press 'n' to reinitialize boids
	//		Press 'q' or 'Q' to change location rule of new boids
	//		Press 'a' or 'A' to step through Boid State menu
	//		Press '/' to show or hide obstacles
	//		// Video-tracking/Optical flow controls:
	//		Press 'f' to show or hide flow lines
	//		Press 'i' to show or hide video image
	//		Press 'h' to show this help message 
	//  See the keyPressed() method for a few more commands
	
	// In this version of the TurtleBoids Sandbox, I am mapping the boids to a torus,
	// in effect the topological figure that results if we assume that boids that go beyond
	// the right edge reappear on the left, boids that go beyond the bottom edge reappear
	// on the top, and vice versa. See the Boids class for details. You can turn the torus 
	// topology off and on with the 't' key. To test the different topologies, press the mouse button
	// near display edges and note how the boids travel to reach your position on a plane and on a torus.
	
	// Every time a boid crosses a display bound its current line is drawn to an offscreen buffer 
	// and a new line is started (in a shifted color). Toggling drawing off will also draw to
	// the offscreen buffer, which is drawn to the screen before any active lines. 
	// When immediate display is off ('r' key), you won't see the line the boid 
	// is currently drawing until it crosses a display bound or until drawing is toggled off. 
	// Immediate display is on when the program starts up, so you will see boids drawing. 
	// Turning it off will increase execution speed notably.
	// Drawing can be turned off and on by pressing the 'd' key. 
	// The effects of color shifitng ('c' key) are best seen when immediate display is on.

	// The parameters that control boid behavior, e.g., sep (separation), align (alignment) and coh (cohesion),
	// can be modified interactively in several ways. 
	// In the Control Panel (spacebar shortcut), when the Use Scaling Factors checkbox is checked, the Alignment
	// and Cohesion fields will be locked, and will be set by multiplying the Separation value by the
	// Alignment Factor and the Cohesion Factor. If the Use Scaling Factors checkbox is NOT checked,
	// you can drag on the Alignment and Coherence number boxes to set those values independent of the separation.
	// Separation can also be incremented or decremented by pressing the '+' and '-' keys. If the Use Scaling Factors
	// checkbox is checked, the alignment and cohesion values will also change. 

	Flock flock;
	boolean flockIsDrawing;
	boolean flockIsDisplaying = true;
	RandUtil rando;
	int targetHue = 60;
	BoidCallbackINF responder;
	VideoCallbackINF videoResponder;
	public PGraphics pg;
	// Control Panel
	ControlP5 controlP5;
	ControlGroup settings;
	Tab settingsTab;
	// Flock Parameters
	float sepFac = 55.0f;
	float alignFac = 233.0f;
	float cohFac = 144.0f;
	float sep = 20;
	float align = sep * alignFac;
	float coh = sep * cohFac;
	boolean useFactors = false;
	float mass;
	float weight = 1.5f;
	int numberOfBoids = 8;
	int totalBoids = 233;
	int meanBoids = 233;
	int maxBoids = 1024;
	int minBoids = 28;
	boolean isPaused = false;
	boolean isShowBoids = true;
	boolean isShowVideo = true;
	boolean isVideoReady = false;
	float avoidance;
	// wind parameters
	boolean isWindy = false;
	float t = 0; 
	float divisor = 128;
	float inc = PApplet.PI/divisor;
	float windspeed = 8.0f;
	float maxWind = 12.0f;
	float minWind = 0.01f;
	float windIncrement = 0.25f;
	float turbulenceIncrement = 0.001f;
	public static final float PHI = (1 + (float) Math.sqrt(5)) * 0.5f;
	public ArrayList<GroupComponent> gList;
	public int step = 0;
	boolean stopOnEdge = false;
	boolean beginOnEdge = false;
	boolean dashedLine = false;
	boolean neonEffect = false;
	// CIRCLE, BLUENOISE, GRIDTRIPLETS, etc....
	enum BoidPlacement {GRID, CENTER, RANDOM, RANDOMCENTER, CENTERGRID, GRIDTRIPLETS, BLOB, BIGBLOB;}
	BoidPlacement placement = BoidPlacement.RANDOM;
	ArrayList<BoidState> boidStateList;
	OpticalFlower optical;
	int videoWidth;
	int videoHeight;
	int displayWidth;
	int displayHeight;
	PImage maskImage;
	boolean isAutoRun = true;
	int selectedBoidState = 0;
	PImage glitchImage;
	BlueStyle obstacles;
	/** threshold at which flow vector magnitude squared triggers drawing */
	float flowMagThresh = 100;
	
	String filePath = "/Users/paulhz/Desktop/Eclipse_output/boids";
	String basename = "vtboids";
	int fileCount = 1;

	IgnoCodeLib igno;
	
	String[] videoDevices;
	int boidTrailsMax = 1;

	
	/**
	 * Method required for Eclipse IDE and standalone Java.
	 * @param args
	 */
	public static void main(String[] args) {
		PApplet.main(new String[] { "--present", "Flocking05" });
	}

	public void setup() {
		// for "best" results, make displayWidth/displayHeight == videoWidth/videoHeight
		// other proportions will distort video but can be useful
		size(1280, 720);
		smooth();
		frameRate = 15;
		displayWidth = width;
		displayHeight = height;
		videoWidth = 640;
		videoHeight = 360;
		pg = createGraphics(width, height);    // offscreen graphics 
		pg.beginDraw();
		pg.smooth();
		pg.background(255);
		pg.endDraw();
		rando = new RandUtil();                // random number utility
		responder = new Responder();           // callback object for TurtleBoid instances
		flock = new Flock();                   // Add an initial set of boids into the system
		flockIsDrawing = true;
		flockIsDisplaying = true;
		initBoidStateList();                   // create up a menu of different sets of cohesion, separation, and alignment values
		// assignBoidState(rando.randomInRange(0, boidStateList.size() - 1), 0.8f);
		// start out with specific separation, alignment and cohesion values for boids
		sep = 21; align = 47; coh = 89;
		sepFac = sep; alignFac = align; cohFac = coh;
		useFactors = true;
		placement = BoidPlacement.values()[rando.randomInRange(0, BoidPlacement.values().length - 1)];
		println("placement = "+ placement.toString());
		// placement = BoidPlacement.CENTER;
		controlP5 = new ControlP5(this);       // initialize ControlP5
		loadPanel();                           // load our control panel
		// we _usually_ want to show the panel to begin with, to pick a video device, 
		// but not in an automated installation
		controlP5.hide();
		printHelp();
		// println("PHI = "+ PHI);
		println("Please choose a video capture device from the popup menu.");
		if (!isVideoReady) { 
			// you'll need to figure out your own values for setupVideo, especially the camera name
			// a call to printDevices can tell you what you have available.
			setupVideo(videoWidth, videoHeight, 30, 8, 0.25f, "FaceTime HD Camera");
			if (!isVideoReady) exit();
		}
		gList = new ArrayList<GroupComponent>();
		initMask();
		// initBlueNoise();
		initBoids();
		// negative avoidance values attract (the range -0.33 to -0.9 works well), 
		// positive values repel (most notable above 1.0): experiment!
		avoidance = -1.25f;
		glitchImage = loadImage("../clouds.jpg");
		glitchImage = loadImageAlpha(glitchImage, 127);
		igno = new IgnoCodeLib(this);
	}
	
//	public boolean sketchFullScreen() {
//		return true;
//	}
	
	/**
	 * @param w          width of video capture
	 * @param h          height of video capture
	 * @param fps        frames per second of video capture, 15 or 30 will do fine
	 * @param grid       edge of a grid cell subdividing video 
	 * @param timespan   time window in decimal seconds to track motion flow (prediction interval)
	 * @param device     name of the video capture device, will vary with the gear you are using
	 */
	public void setupVideo(int w, int h, int fps, int grid, float timespan, String device) {
		// printDevices();
		// optical = new OpticalFlower(this, width, height, 20, 0.5f, "IIDC FireWire Video");
		// optical = new OpticalFlower(this, width, height, 20, 0.5f, "USB Video Class Video");
		// optical = new OpticalFlower(this, width, height, 20, 0.5f);
		// optical = new OpticalFlower(this, width, height, 20, 0.5f, Capture.list()[0]);
		// optical = new OpticalFlower(this, width, height, 20, 0.5f, "Built-in iSight");
		if (null == optical) {
			optical = new OpticalFlower(this, w, h, fps, grid, timespan, device);
		}
		else {
			optical.setVideoParams(w, h, fps, grid, timespan, device);
		}
		isVideoReady = optical.init();
		if (!isVideoReady) {
			println("***** Video failed to initialize. Please check that you are using an available device and settings. *****");
			println("***** Select device and settings from the following list: *****");
			printDevices();
		}
		else {
			optical.setFlowColor(color(233, 220, 199, 127));
			optical.setImageFlowColor(color(144, 110, 233, 255));
			if (isShowVideo) {
				optical.showImage();
			}
			else {
				optical.hideImage();
			}
			videoResponder = new VideoResponder();
			optical.setResponder(videoResponder);
		}
	}
	
	/**
	 * @return   a list of attached video devices and information about each
	 */
	public String[] getVideoDevices() {
		if (null == videoDevices) {
			videoDevices = Capture.list();
		}
		return videoDevices;
	}
	
	/**
	 * Prints available video device information to the console.
	 */
	public void printDevices() {
		PApplet.println("Available Video Devices:");
		String[] devices = getVideoDevices();
		for (String device : devices) {
			PApplet.println("-- "+ device);
		}
	}

	/**
	 * Sets up list of BoidStates (separation, alignment, cohesion) used by a ControlP5 menu.
	 */
	public void initBoidStateList() {
		boidStateList = new ArrayList<BoidState>();
		// sep = 89; align = 34; coh = 21;  // twists
		boidStateList.add(new BoidState(89, 34, 21, "twists"));		
		// sep = 13; align = 8; coh = 144;  // raggedy flocks
		boidStateList.add(new BoidState(13, 8, 144, "ragged"));
		// sep = 55; align = 34; coh = 21;  // open tangles with paired and tripled boids
		boidStateList.add(new BoidState(55, 34, 21, "open tangles"));
		// sep = 55; align = 34; coh = 26;  // more clusters of boids
		boidStateList.add(new BoidState(55, 34, 26, "clusters"));
		// sep = 89; align = 29; coh = 76;  // 
		boidStateList.add(new BoidState(89, 29, 76, "isolated drunks"));
		// sep = 47; align = 29; coh = 76;  // spurts of aligned flocks that don't continue
		boidStateList.add(new BoidState(47, 29, 76, "aligned spurts"));
		// sep = 21; align = 47; coh = 123; // flocks joining and separating, swirling
		boidStateList.add(new BoidState(21, 47, 123, "swirling")); 
		// sep = 5; align = 8; coh = 377;   // skeins, if the wind don't blow steady
		boidStateList.add(new BoidState(5, 8, 610, "skeins"));
		// sep = 21; align = 18; coh = 233;   // draws tangled, open skeins, if the wind don't blow too hard
		boidStateList.add(new BoidState(21, 18, 240, "tangled skeins"));
		// sep = 76; align = 34; coh = 89;
		boidStateList.add(new BoidState(76, 34, 89, "untitled 2"));
		// sep = 144; align = 89; coh = 377;
		boidStateList.add(new BoidState(144, 89, 377, "untitled 3"));
		// sep = 144; align = 110; coh = 89;
		boidStateList.add(new BoidState(144, 110, 89, "untitled 4"));
		// sep = 144; align = 233; coh = 89;
		boidStateList.add(new BoidState(199, 233, 89, "wide tracer "));
		// sep = 13; align = 47; coh = 34;
		boidStateList.add(new BoidState(13, 47, 34, "small clusters"));
		// sep = 21; align = 47; coh = 55;
		boidStateList.add(new BoidState(21, 47, 55, "wavy"));
		// sep = 34; align = 55; coh = 89;
		boidStateList.add(new BoidState(34, 55, 89, "wide wavy"));
		// sep = 13; align = 108; coh = 123;
		boidStateList.add(new BoidState(13, 108, 123, "tight join"));
		// sep = 21; align = 108; coh = 123;
		boidStateList.add(new BoidState(21, 108, 123, "untitled 9"));
		// sep = 34; align = 108; coh = 123;
		boidStateList.add(new BoidState(34, 108, 123, "untitled 10"));
		// sep = 2; align = 3; coh = 21;
		boidStateList.add(new BoidState(2, 3, 21, "tight flocks"));
	}
	
	/**
	 * @param i       index of desired BoidState in the boidStateList
	 * @param scale   scale by which to multiply separation, alignment and cohesion values in BoidState
	 */
	public void assignBoidState(int i, float scale) {
		if (i >= boidStateList.size()) {
			i = i % boidStateList.size();
		}
		else if (i < 0) {
			i = boidStateList.size() - (-i % boidStateList.size());
		}
		BoidState state = boidStateList.get(i);
		boolean oldUseFactors = useFactors;
		useFactors = false;
		Numberbox n1 = (Numberbox) controlP5.getController("setSeparation");
		Numberbox n2 = (Numberbox) controlP5.getController("setAlignment");
		Numberbox n3 = (Numberbox) controlP5.getController("setCohesion");
		n1.setValue(max((int)(state.sep * scale), 1));
		n2.setValue(max((int)(state.align * scale), 1));
		n3.setValue(max((int)(state.coh * scale), 1));
		Numberbox n0 = (Numberbox) controlP5.getController("setSepFac");
		Numberbox n4 = (Numberbox) controlP5.getController("setAlignFac");
		Numberbox n5 = (Numberbox) controlP5.getController("setCohFac");
		n0.setValue(sep);
		n4.setValue(align);
		n5.setValue(coh);
		useFactors = oldUseFactors;
		println("assigned sep = "+ sep +", align = "+ align +", coh = "+ coh + "  \n boid state "+ i +" -- "+ state.name);
	}
	
	/**
	 * Just some code to help test statistical accuracy of a quick gaussian method.
	 */
	public void testGauss() {
		double sum = 0;
		double max = Double.MIN_VALUE, min = Double.MAX_VALUE;
		int iter = 20;
		double num;
		for (int i = 0; i < iter; i++) {
			num = rando.gauss(width/2, width/2);
			println(num);
			if (num < min) min = num;
			if (num > max) max = num;
			sum += num;
		}
		println("average = "+ sum/iter +", max = "+ max +", min = "+ min);
	}
	
	/**
	 * Intializes and deploys a new flock of boids to the display. 
	 * Location of boids is determined by the placement variable, of type BoidPlacement.
	 */
	public void initBoids() {
		int[] hues = {0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330};
		targetHue = rando.randomElement(hues);
		for (int i = 0; i < totalBoids; i++) {
			addOneBoid(random(0, width), random(0, height));
		}
		setBoidDrawStyle(flock.getBoids());
		placeBoids(flock.getBoids());
	}
	
	public void setBoidDrawStyle(Boid tBoid) {
		// auto draw over selected distance with a call to setMaxDistance
		// int[] nums = {21, 34, 55, 89};
		int[] nums = {89, 89, 89, 89, 89, 89, 144, 144, 144, 233, 233, 377};
		((TurtleBoid) tBoid).setMaxDistance(rando.randomElement(nums));
	}
	
	public void setBoidDrawStyle(ArrayList<Boid> someBoids) {
		for (Boid tBoid : someBoids) {
			setBoidDrawStyle(tBoid);
		}
	}
	
	public void placeBoids(ArrayList<Boid> someBoids) {
		println("placement = "+ placement.toString());
		switch (placement) {
		case RANDOM: {
			for (Boid tBoid : someBoids) {
				PVector loc = new PVector(random(0, width), random(0, height));
				tBoid.setLoc(loc);
			}
			break;
		}
		case CENTER: {
			for (Boid tBoid : someBoids) {
				tBoid.setLoc(new PVector(width/2, height/2));
			}
			break;
		}
		case RANDOMCENTER: {
			int xctr = -1;
			while ((xctr < 0) || (xctr > width)) {
				xctr = (int) rando.gauss(width/2, width * 24);
			}
			int yctr = -1;
			while ((yctr < 0) || (yctr > height)) {
				yctr = (int) rando.gauss(height/2, height * 24);
			}
			println("xctr = "+ xctr +", yctr = "+ yctr);
			for (Boid tBoid : someBoids) {
				int x = (int) rando.gauss(xctr, width);
				int y = (int) rando.gauss(yctr, height);
				tBoid.setLoc(new PVector(x, y));
			}
			break;
		}
		case GRID: {
			int q = 1;
			while (q * q < someBoids.size()) q++;
			println("Grid with "+ q +" rows and "+ q +" columns");
			float xStep = (float) Math.floor(width/q);
			float yStep = (float) Math.floor(height/q);
			float xStart = xStep/2.0f;
			float yStart = yStep/2.0f;
			ArrayList<PVector> gridPoints = new ArrayList<PVector>(q * q);
			for (int i = 0; i < q; i++) {
				for (int j = 0; j < q; j++) {
					gridPoints.add(new PVector(xStart + i * xStep, yStart + j * yStep));
				}
			}
			rando.shuffle((ArrayList)gridPoints);
			int i = 0;
			for (Boid tBoid : someBoids) {
				tBoid.setLoc(gridPoints.get(i++));
			}
			break;
		}
		case CENTERGRID: {
			int q = 1;
			while (q * q < someBoids.size()) q++;
			println("Grid with "+ q +" rows and "+ q +" columns");
			float xStep = (float) Math.floor(width/(q * 2));
			float yStep = (float) Math.floor(height/(q * 2));
			float xStart = xStep/2.0f + 0.25f * width;
			float yStart = yStep/2.0f + 0.25f * height;
			ArrayList<PVector> gridPoints = new ArrayList<PVector>(q * q);
			for (int i = 0; i < q; i++) {
				for (int j = 0; j < q; j++) {
					gridPoints.add(new PVector(xStart + i * xStep, yStart + j * yStep));
				}
			}
			rando.shuffle((ArrayList)gridPoints);
			int i = 0;
			for (Boid tBoid : someBoids) {
				tBoid.setLoc(gridPoints.get(i++));
			}
			break;
		}
		case GRIDTRIPLETS: {
			int q = 1;
			int limit = someBoids.size()/3;
			while (q * q < limit) q++;
			println("Grid with "+ q +" rows and "+ q +" columns");
			float xStep = (float) Math.floor(width/q);
			float yStep = (float) Math.floor(height/q);
			float xStart = xStep/2.0f;
			float yStart = yStep/2.0f;
			ArrayList<PVector> gridPoints = new ArrayList<PVector>(q * q);
			for (int i = 0; i < q; i++) {
				for (int j = 0; j < q; j++) {
					gridPoints.add(new PVector(xStart + i * xStep, yStart + j * yStep));
				}
			}
			rando.shuffle((ArrayList)gridPoints);
			for (int i = 0; i < limit; i++) {
				PVector loc = gridPoints.get(i);
				for (int j = 0; j < 3; j++) {
					Boid tBoid = someBoids.get(i * 3 + j);
					tBoid.setLoc(new PVector(loc.x, loc.y));					
				}
			}
			break;
		}
		case BLOB: 
		case BIGBLOB: {
			float xctr = width/2f;
			xctr += (float) rando.gauss(0, width/8);
			float yctr = height/2f;
			yctr += (float) rando.gauss(0, height/8);
			BezShape blob = makeBlob(xctr, yctr);
			BezRectangle rect = blob.boundsRect();
			float xScale = 0.55f * width/rect.getWidth();
			float yScale = 0.55f * height/rect.getHeight();
			if (placement == placement.BIGBLOB) {
				xScale = 0.89f * width/rect.getWidth();
				yScale = 0.89f * height/rect.getHeight();
			}
			if (xScale > yScale) blob.scaleShape(yScale);
			else blob.scaleShape(xScale);
			rect = blob.boundsRect();
			float left = rect.getLeft();
			float right = rect.getRight();
			float top = rect.getTop();
			float bottom = rect.getBottom();
			for (Boid tBoid : someBoids) {
				float x = rando.randomInRange(left, right);
				float y = rando.randomInRange(top, bottom);
				while (!blob.containsPoint(x, y)) {
					x = rando.randomInRange(left, right);
					y = rando.randomInRange(top, bottom);
				}
				tBoid.setLoc(new PVector(x, y));
			}
			break;
		}
		default: {
			for (Boid tBoid : someBoids) {
				tBoid.setLoc(new PVector(random(0, width), random(0, height)));
			}
		}
		}		
	}

	
	/**
	 * Initializes a boid and adds it to the flock. Bottleneck method and best place to experiment with boid mods.
	 */
	public Boid addOneBoid(float x, float y) {
		// TurtleBoid tBoid = new TurtleBoid(this, new PVector(width/2, height/2), 5.0f, 0.08f);
		TurtleBoid tBoid = new TurtleBoid(this, new PVector(x, y), 8.0f, 0.8f);
		tBoid.setVisible(isShowBoids);
		tBoid.setResponder(responder);
		float m = (random(0.9f, 1.1f) + random(0.9f, 1.1f) + random(0.9f, 1.1f)) / 3.0f;
		//float m = (random(5.1f, 9.5f) + random(5.1f, 9.5f) + random(5.1f, 9.5f)) / 3.0f;
		//
		tBoid.setSeparationDistance(sep);
		tBoid.setAlignmentDistance(align);
		tBoid.setCohesionDistance(coh);
		//
		/*
		float separation = (float) rando.gauss(sep, 0.005);
		if (separation > 0) tBoid.setSeparationDistance(separation);
		else tBoid.setSeparationDistance(sep);
		float alignment = (float) rando.gauss(align, 0.005);
		if (alignment > 0) tBoid.setAlignmentDistance(alignment);
		else tBoid.setAlignmentDistance(align);
		float cohesion = (float) rando.gauss(coh, 0.005);
		if (cohesion > 0) tBoid.setCohesionDistance(cohesion);
		else tBoid.setCohesionDistance(coh);
		*/
		tBoid.setDisplaying(flockIsDisplaying);
		tBoid.setMaxDistance(boidMaxDistance());
		Turtle t = tBoid.getTurtle();
		// fewer trails, less memory required
		t.setMaxTrails(boidTrailsMax);
		// line weight
		t.setWeight(weight);
		float w = (float) rando.gauss(weight, 0.01);
		if (w > 0) t.setWeight(w);
		t.setNoFill();
		setShapeStroke(tBoid, targetHue, 30);
		if (!flockIsDrawing) t.penUp();
		flock.addBoid(tBoid);
		return tBoid;
	}
	

	/**
	 * @return   a float to serve as maxDistance for a Turtle
	 */
	float boidMaxDistance() {
		float[] distanceList = {233, 377, 610, 987, 1536};
		return rando.randomElement(distanceList);
	}
	
	 float[] bluenoise = {116.002f, 552.909f, 795.396f, 588.671f, 1522.72f, 549.958f,
			 1601.45f, 301.583f, 456.625f, 520.227f, 829.933f, 423.29f, 625.748f, 92.9207f,
			 1127.25f, 433.887f, 48.4038f, 134.565f, 223.59f, 283.084f, 1079.5f, 247.117f,
			 1238.43f, 599.472f, 1648.33f, 44.4122f, 1084.71f, 82.9729f, 633.237f, 518.39f,
			 260.96f, 476.742f, 726.323f, 242.086f, 1298.8f, 153.001f, 56.3434f, 332.905f,
			 1255.27f, 332.292f, 1731.34f, 194.366f, 1002.54f, 530.887f, 1803.97f, 529.137f,
			 1688.77f, 432.702f, 591.905f, 356.409f, 1430.28f, 342.82f, 389.749f, 329.543f,
			 227.595f, 102.65f, 380.075f, 49.8877f, 883.949f, 106.836f, 915.726f, 290.936f,
			 1375.25f, 527.002f, 483.907f, 192.45f, 1476.03f, 153.935f};
	 ArrayList<PVector> blueVectors = new ArrayList<PVector>();
	 float[] blueForce = new float[bluenoise.length/2];
	 // float[] blueForceValues = {76, 89, 110, 123, 144, 152, 178, 199, 233, 246, 288, 322, 377};
	 // float[] blueForceValues = {21, 29, 34, 47, 55, 76, 89, 110, 123, 144};
	 // float[] blueForceValues = {13, 21, 29, 34, 47, 55, 76, 89, 110, 123};
	 float[] blueForceValues = {21, 34, 55};
	 boolean isUseBlue = false;
	 enum BlueStyle {BLUE, CIRCLE, BLOB, CORNERBLOB, OPENBLOB;}
	 /**
	 * Intializes locations that attract or repel boids. 
	 * Locations may be determined by blue noise or any other distribution you like.
	 * Experimental code.
	 */
	 public void initBlueNoise() {
		 if (null == obstacles) obstacles = BlueStyle.BLOB;
		 blueVectors.clear();
		 switch (obstacles) {
		 case BLUE: {
			 float hscale = width * 0.00055f;
			 float vscale = width * 0.0005f;
			 for (int i = 0; i < bluenoise.length/4; i++) {
				 float x = bluenoise[2 * i] * hscale;
				 float y = bluenoise[2 * i + 1] * vscale + height/4.0f;
				 blueVectors.add(new PVector(x, y));
				 // println("blue vectors element "+ i +" = "+ x +", "+ y);
			 }
			 break;
		 }
		 case CIRCLE: {
			 float k = (float) rando.gauss(0.375, 0.0125);
			 float radius = width > height ? k * height : k * width;
			 int sides = 123;
			 float xctr = width/2f;
			 float yctr = height/2f;
			 double ang = GeomUtils.TWO_PI/sides;
			 Matrix3 matx = new Matrix3();
			 matx.translateCTM(-xctr, -yctr);
			 matx.rotateCTM(ang);
			 matx.translateCTM(xctr, yctr);
			 Point2D.Double pt = new Point2D.Double(xctr, yctr - radius);
			 for (int i = 0; i < sides; i++) {
				 pt = matx.multiplyPointByNormalCTM(pt.x, pt.y, pt);
				 blueVectors.add(new PVector((float) (pt.getX() + rando.gauss(0, 128)), (float) (pt.getY() + rando.gauss(0, 128))));
			 }
			 break;
		 }
		 case BLOB: 
		 case CORNERBLOB: {
			 float xctr = width/2f;
			 xctr += (float) rando.gauss(0, width/8);
			 float yctr = height/2f;
			 yctr += (float) rando.gauss(0, height/8);
			 if (obstacles == BlueStyle.CORNERBLOB) {
				 xctr += width/2f;
				 yctr += height/2f;
			 }
			 BezShape blob = makeBlob(xctr, yctr);
			 BezRectangle rect = blob.boundsRect();
			 float xScale = 0.75f * width/rect.getWidth();
			 float yScale = 0.75f * height/rect.getHeight();
			 blob.scaleShape(xScale, yScale);
			 blob.asPolygon(16);
			 float[] xcoords = blob.xcoords();
			 float[] ycoords = blob.ycoords();
			 for (int i = 0; i < xcoords.length; i++) {
				 blueVectors.add(new PVector(xcoords[i], ycoords[i]));
			 }
			 break;
		 }
		 case OPENBLOB: {
			 float xctr = width/2f;
			 xctr += (float) rando.gauss(0, width/8);
			 float yctr = height/2f;
			 yctr += (float) rando.gauss(0, height/8);
			 BezShape blob = makeBlob(xctr, yctr);
			 BezRectangle rect = blob.boundsRect();
			 float xScale = 0.75f * width/rect.getWidth();
			 float yScale = 0.75f * height/rect.getHeight();
			 blob.scaleShape(xScale, yScale);
			 blob.asPolygon(12);
			 float[] xcoords = blob.xcoords();
			 float[] ycoords = blob.ycoords();
			 int ct = xcoords.length;
			 int start = 2 * (int) Math.floor(rando.randomInRange(0, ct)/2f);
			 int len = (int) rando.randomInRange(ct/8, ct/4);
			 int end = start + len;
			 for (int i = 0; i < xcoords.length; i++) {
				 if ((i > start) && (i < end)) continue;
				 blueVectors.add(new PVector(xcoords[i], ycoords[i]));
			 }
			 break;
		 }
		 default: {
			 
		 }
		 }
		 isUseBlue = true;
	 }
	 
	 /**
	 * Initializes maskImage, which is use to fade out the video capture image when it is displayed unde the boids. 
	 */
	public void initMask() {
		 maskImage = this.createImage(width, height, ARGB);
		 int maskColor = color(64, 64, 64, 64);
		 maskImage.loadPixels();
		 for (int i = 0; i < width * height; i++) {
			 maskImage.pixels[i] = maskColor;
		 }
	 }
	
	/**
	 * Prints some helpful (!) information to the console.
	 */
	public void printHelp() {
		println("Press spacebar to show or hide controls");
		println("Press 'd' to toggle drawing");
		println("Press 'r' to toggle immediate display of drawing");
		println("Press 'c' to shift color");
		println("Press 'x' to erase");
		println("Press 'p' to pause");
		println("Press 's' to save to AI 7.0 format file");
		println("Press 'v' to toggle visibility of boids");
		println("Press 'l' to toggle cohesion lines (best with few boids)");
		println("Press 't' to toggle topology from torus to plane");
		println("Press 'w' to toggle wind");
		println("Press '+' or '-' to increase or decrease boids spearation");
		println("Press UP or DOWN arrow keys to increase or decrease wind force");
		println("Press LEFT or RIGHT arrow keys to turn boids.");
		println("Press mouse to attract boids (when controls are hidden)");
		println("Drag mouse to push or pull boids (when controls are hidden)");
		println("Press 'g' to toggle stop drawing at edge");
		println("Press 'b' to toggle start drawing at edge");
		println("Press 'n' to reinitialize boids");
		println("Press 'q' or 'Q' to change location rule of new boids");
		println("Press 'a' or 'A' to step through Boid State menu");
		println("Press '/' to show or hide obstacles");
		// Video-tracking/Optical flow controls:
		println("Press 'f' to show or hide flow lines");
		println("Press 'i' to show or hide video image");
		println("Press 'h' to show this help message"); 
	}

	// TODO overlay all images in correct order (maybe this is already done? not sure /2016/ ).
	public void draw() {
		// if isPaused, exit loop instead of calling noLoop() so P5 and other calls can still propagate
		if (isPaused) return;
		if (!isShowVideo) {
			// already done
			// PImage img = loadImageAlpha(glitchImage, 127);
			if (width == pg.width && height == pg.height) {
				background(pg);
			}
			else {
				pg.resize(width, height);
				background(pg);
				println("width = "+ width +", height = "+ height);
				println("pg.width = "+ pg.width +", pg.height = "+ pg.height);
			}
			image(glitchImage, 0, 0, width, height);
		}
		if (isVideoReady) {
			optical.flow();
			if (!optical.isFlagflow()) {
				this.drawVectorLines();
			}
		}
		if (isUseBlue) {
			for (PVector vec : blueVectors) {
				BezCircle bez = BezCircle.makeCenterRadius(vec.x, vec.y, 5);
				bez.setFillColor(color(233, 89, 21));
				bez.setNoStroke();
				bez.draw();
			}
		}
		runBoids();
	}
	
	// @TODO There's a lot of extraneous code in runBoids(), experiments that haven't been pulled or broken out yet.
	/**
	 * Steps the boid through one frame of motion.
	 */
	public void runBoids() {
		if (isWindy) {
			Boid boid = flock.getBoids().get(0);
			if (null != boid) {
				PVector v = new PVector(cos(t), sin(t));
				v.mult((float) rando.gauss(2.0, 0.1));
				PVector shake = new PVector(random(-1, 1), random(-1, 1));
				shake.mult((float) rando.gauss(2, 0.2));
				v.add(shake);
				v.mult(windspeed);
				blowWind(-width/2, -height/2, v);
			}
		}
		if (mousePressed && !controlP5.isVisible()) {
			PVector vec = new PVector(mouseX, mouseY);
			for (Boid tBoid : flock.getBoids()) {
				PVector closestImage = tBoid.nearestPointOnTorus(tBoid.getLoc(), vec);
				// negative avoidance force = attraction
				tBoid.avoid(closestImage, -0.5f);
				// tBoid.arrive(new PVector(mouseX, mouseY));
			}
		}
		flock.run();
		t += inc;
		if (divisor < 4096 && isWindy) divisor++;
		evitar();
		// deleted dashed line code here
		inc = PApplet.PI/divisor;
		// step++;
		// fade the background
		if (frameCount % 30 == 0) {
			pg.fill(255, 10);
			pg.noStroke();
			pg.rect(0, 0, width, height);
		}
		// autorun switches state from time to time
		// TODO give autorun a separate method
		if (isAutoRun && frameCount % 480 == 0) {
			float rand = random(0,1);
			if (rand > 0.5f) {
				erase();
				if (rand > 0.5f) {
					placement = BoidPlacement.values()[rando.randomInRange(0, BoidPlacement.values().length - 1)];
					println("placement = "+ placement.toString());
					newBoids();
				}
				assignBoidState(rando.randomInRange(0, boidStateList.size() - 1), 1f);
			}
			else if (rand < 0.5f) {
				assignBoidState(rando.randomInRange(0, boidStateList.size() - 1), 1f);
			}
			else if (rand < 0.33) {
				if (totalBoids > meanBoids) {
					subtractBoids();
				}
				else if (totalBoids < meanBoids) {
					addBoids();
				}
			}
		}
	}
	
	/**
	 * Adds attraction/repulsion forces to boids, from the video optical flow
	 * and possibly from the blueVectors array. 
	 */
	public void evitar() {
		if (!isVideoReady) return;
		// add in the vectors from video motion flow
		for (Boid tBoid : flock.getBoids()) {
			PVector vec = optical.getFlow(tBoid.getLoc());
			// tBoid.avoid(vec, avoidance);
			tBoid.applyForce(vec);
			// activate drawing if vec magnitude exceeds threshold
			if ( vec.magSq() > flowMagThresh && !((TurtleBoid) tBoid).getTurtle().isPenDown() ) {
				((TurtleBoid) tBoid).getTurtle().penDown();
				((TurtleBoid) tBoid).setDistance(0);
				((TurtleBoid) tBoid).setMaxDistance(boidMaxDistance());
				// println("---- force activated pen ----");
			}
		}
		if (isUseBlue) {
			for (Boid tBoid : flock.getBoids()) {
				for (PVector vec : blueVectors) {
					float spacer = rando.randomElement(blueForceValues);
					PVector closestImage = tBoid.nearestPointOnTorus(tBoid.getLoc(), vec);
					if (PVector.dist(tBoid.getLoc(), closestImage) < spacer) {
						tBoid.avoid(closestImage, avoidance);
					}
				}
			}
		}
	}

	public void mousePressed() {
		if (controlP5.isVisible()) return;
		// when the control panel is not visible, a mouse press attracts the boids. 
		for (Boid tBoid : flock.getBoids()) {
			tBoid.arrive(new PVector(mouseX, mouseY));
		}
	}
	
	public void mouseDragged() {
		if (controlP5.isVisible()) return;
		// when the control panel is not visible, a mouse drags pulls on the boids. 
		PVector wind = new PVector(mouseX - pmouseX, mouseY - pmouseY);
		wind.mult(0.1f);
		blowWind(mouseX, mouseY, wind);
	}
	
	/**
	 * @param x      x-coordinate of origin of wind force
	 * @param y      y-coordinate of origin of wind force
	 * @param wind   a PVector for wind direction and magnitude of wind force
	 */
	public void blowWind(float x, float y, PVector wind) {
		PVector loc = new PVector(x, y);
		for (Boid tBoid : flock.getBoids()) {
			float dist = PVector.dist(loc, tBoid.getLoc());
			float k = 1.0f / (dist * dist);
			wind.mult(0.90f + k);
			tBoid.applyForce(wind);
		}
	}
	
	/* (non-Javadoc)
	 * handles key presses intended as commands
	 * @see processing.core.PApplet#keyPressed()
	 */
	public void keyPressed() {
		parseKey(key, keyCode);
	}

	// parse key presses TODO
	public void parseKey(char key, int keyCode) {
		if (key == CODED) {
			if (keyCode == LEFT) {
				for (Boid tBoid : flock.getBoids()) {
					tBoid.turn(radians(-9));
				}
			}
			else if (keyCode == RIGHT) {
				for (Boid tBoid : flock.getBoids()) {
					tBoid.turn(radians(9));
				}
			}
			else if (keyCode == UP) {
				if (keyCode == SHIFT) println("shifted up");
				windspeed += windIncrement;
				if (windspeed > maxWind) windspeed = maxWind;
			}
			else if (keyCode == DOWN) {
				if (keyCode == SHIFT) println("shifted down");
				windspeed -= windIncrement;
				if (windspeed <= minWind) windspeed = minWind;
			}
		} 
		else {
			decode(key);
		}
	}
	
	/**
	 * associates characters input from keyboard with commands
	 * @param ch   a char value representing a command
	 */
	public void decode(char ch) {
		if (key == 'd' || key == 'D') {
			toggleDrawing();
			println("trail drawing is "+ flockIsDrawing);
		}
		else if (key == 'r' || key == 'R') {
			flockIsDisplaying = !flockIsDisplaying;
			for (Boid boid : flock.getBoids()) {
				((TurtleBoid) boid).setDisplaying(flockIsDisplaying);
			}
			println("trail displaying is "+ flockIsDisplaying);
		}
		else if (key == 'c' || key == 'C') {
			for (Boid tBoid : flock.getBoids()) {
				Turtle t = ((TurtleBoid)tBoid).getTurtle();
				int tbStrokeColor = t.strokeColor();
				colorMode(HSB, 360, 100, 100);
				float targHue = hue(tbStrokeColor) + 7.5f;
				colorMode(RGB, 255, 255, 255);
				setShapeStroke(((TurtleBoid) tBoid), targHue, 30);
			}
		}
		else if (key == 'x') {
			erase();
			gList.clear(); 
		}
		else if (key == 'X') {
			bgErase(32);
			// gList.clear();
		}
		else if (key == 'p' || key == 'P') {
			isPaused = !isPaused;
		}
		else if (key == 's' || key == 'S') {
			stopDrawing();
			saveListAI(basename);
		}
		else if (key == 'y') {
			// reset the obstacles
			initBlueNoise();
		}
		else if (key == 'Y') {
			obstacles = BlueStyle.values()[(obstacles.ordinal() + 1) % BlueStyle.values().length];
			// reset the obstacles
			initBlueNoise();
			println("obstacles = "+ obstacles.toString());					
		}
		else if (key == '+' || key == '=') {
			// trigger call to setSeparation() by setting the number box, avoid recursion
			Numberbox n1 = (Numberbox) controlP5.getController("setSeparation");
			n1.setValue(sep + 1);
			adjustFlock();
		}
		else if (key == '-' || key == '_') {
			// trigger call to setSeparation() by setting the number box, avoid recursion
			Numberbox n1 = (Numberbox) controlP5.getController("setSeparation");
			n1.setValue(sep - 1);
			adjustFlock();
		}
		else if (key == 'w' || key == 'W') {
			isWindy = !isWindy;
			if (isWindy) println("wind is blowing at windspeed "+ windspeed);
			else println("wind is calm");
		}
		else if (key == ' ') {
			if (controlP5.isVisible()) {
				controlP5.hide();
			}
			else {
				controlP5.show();
			}
		}
		else if (key == 'v' || key == 'V') {
			isShowBoids = !isShowBoids;
			for (Boid tBoid : flock.getBoids()) {
				tBoid.setVisible(isShowBoids);
			}
		}
		else if (key == 't' || key == 'T') {
			boolean mapToTorus = !Boid.isMapToTorus();
			Boid.setMapToTorus(mapToTorus);
			if (mapToTorus) println("Mapping to torus");
			else println("Mapping to plane");
		}
		else if (key == 'l' || key == 'L') {
			boolean drawLines = !Boid.isDrawCohesionLines();
			Boid.setDrawCohesionLines(drawLines);
		}
		else if (key == 'g' || key == 'G') {
			stopOnEdge = !stopOnEdge;
			println("stop on edge = "+ stopOnEdge);
		}
		else if (key == 'b' || key == 'B') {
			beginOnEdge = !beginOnEdge;
			println("begin on edge = "+ beginOnEdge);
		}
		else if (key == 'm' || key == 'M') {
			testGauss();
		}
		else if (key == 'n') {
			newBoids();
		}
		else if (key == 'N') {
			toggleDrawing();
			toggleDrawing();
			placeBoids(flock.getBoids());
		}
		else if (key == 'o' || key == 'O') {
			neonEffect = !neonEffect;
			println("neon effect = "+ neonEffect);
		}
		else if (key == 'e' || key == 'E') {
			dashedLine = !dashedLine;
		}
		else if (key == 'h' || key == 'H') {
			printHelp();
		}
		else if (key == 'f' || key == 'F') {
			if (isVideoReady) optical.toggleFlowDisplay();
		}
		else if (key == 'i' || key == 'I') {
			if (!isVideoReady) return;
			isShowVideo = !isShowVideo;
			if (isShowVideo) {
				optical.showImage();
			}
			else {
				optical.hideImage();
			}
		}
		else if (key == 'k' || key == 'K') {
			skipAction = !skipAction;
			println("skipAction = "+ skipAction);
		}
		else if (key == 'a') {
			selectedBoidState = (selectedBoidState + 1) % boidStateList.size();
			assignBoidState(selectedBoidState, 1f);
			// println("selectedBoidState: "+ selectedBoidState);
		}
		else if (key == 'A') {
			selectedBoidState = selectedBoidState == 0 ? boidStateList.size() - 1 : (selectedBoidState - 1) % boidStateList.size();
			assignBoidState(selectedBoidState, 1f);
			// println("selectedBoidState: "+ selectedBoidState);
		}
		else if (key == 'q') {
			//placement = BoidPlacement.values()[rando.randomInRange(0, BoidPlacement.values().length - 1)];
			placement = BoidPlacement.values()[(placement.ordinal() + 1) % BoidPlacement.values().length];
			println("placement = "+ placement.toString());		
		}
		else if (key == 'Q') {
			//placement = BoidPlacement.values()[rando.randomInRange(0, BoidPlacement.values().length - 1)];
			placement = placement.ordinal() == 0 ? BoidPlacement.values()[BoidPlacement.values().length - 1] : BoidPlacement.values()[placement.ordinal() - 1];
			println("placement = "+ placement.toString());		
		}
		else if (key == 'j' || key == 'J') {
			avoidance = -avoidance;
			println("-- avoidance = "+ avoidance);
		}
		else if (key == ';') {
			if (!isVideoReady) return;
			// dump vector magnitudes as a 2D array
			PVector[] flowList = optical.getFlowList();
			println(); println();
			int gh = optical.getGh();
			int gw = optical.getGw();
			for (int iy = 0; iy < gh; iy++) {
				for (int ix = 0; ix < gw; ix++) {
					print(Math.round(flowList[iy * gw + ix].mag()) + "  ");
				}
				println();
			}
			println(); println();
		}
		else if (key == '/') {
			isUseBlue = !isUseBlue;
			println ("isUseBlue = "+ isUseBlue);
		}
	}
	
	/**
	 * Detects Caps Lock state. We use Caps Lock state to switch between audio and graphics command sets. 
	 * @return true if Caps Lock is down, false otherwise.
	 */
	public boolean isCapsLockDown() {
		boolean isDown = Toolkit.getDefaultToolkit().getLockingKeyState(KeyEvent.VK_CAPS_LOCK);
		return isDown;
	}
	
	/**
	 * Executes a supplied commend sequence
	 * @param cmd   a command sequence
	 */
	public void exec(String cmd) {
		char[] cycle = cmd.toCharArray();
		for (char ch : cycle) {
			decode(ch);
		}
	}

	/**
	 * Tells all the boids to start drawing.
	 */
	public void startDrawing() {
		// targetHue = (targetHue + 5) % 360;
		for (Boid tBoid : flock.getBoids()) {
			((TurtleBoid) tBoid).getTurtle().penDown();
			// maintain a more constant hue
			// setShapeStroke(((TurtleBoid) tBoid), targetHue, 30);
		}
	}
	
	/**
	 * Tells all the boids to stop drawing.
	 */
	public void stopDrawing() {
		drawCurrentOffscreen(pg);
		GroupComponent gatherBoids = new GroupComponent(this);
		for (Boid tBoid : flock.getBoids()) {
			Turtle t = ((TurtleBoid) tBoid).getTurtle();
			t.penUp();
			gatherBoids.add(t.getTrails());
			t.clear();
		}
		gList.add(gatherBoids);
		// draws the entire display list for each turtleBoid
		// drawOffscreen(pg);
	}
	
	/**
	 * Turns boid drawing on and off. 
	 * Note that every time you toggle drawing on, you create data for 
	 * a new group component in an exported Illustrator file. 
	 */
	public void toggleDrawing() {
		flockIsDrawing = !flockIsDrawing;
		if (flockIsDrawing) {
			startDrawing();
		}
		else {
			stopDrawing();
		}
	}
	
	
	/**
	 * Draws current trail in TurtleBoid instances to a supplied offscreen buffer.
	 * Does not erase the buffer first, draws on top of other graphics.
	 * @param pg   the offscreen buffer, a PGraphics instance.
	 */
	public void drawCurrentOffscreen(PGraphics pg) {
		// draw offscreen without erasing previous graphics
		pg.beginDraw();
		pg.smooth();
		// just draw the current trails
		for (Boid tBoid : flock.getBoids()) {
			Turtle t = ((TurtleBoid) tBoid).getTurtle();
			if (null != t.getCurrentTrail()) {
				t.getCurrentTrail().draw(pg);
			}
		}		  
		pg.endDraw();
	}

	/**
	 * Draws all geometry stored in TurtleBoid instances to a supplied offscreen buffer.
	 * Erases the buffer first.
	 * @param pg   the offscreen buffer, a PGraphics instance.
	 */
	public void drawOffscreen(PGraphics pg) {
		// println("offscreen starting...");
		pg.beginDraw();
		pg.smooth();
		pg.background(255);
		/*
		  for (Boid tBoid : flock.getBoids()) {
		    Turtle t = ((TurtleBoid) tBoid).getTurtle();
		    t.draw(pg);
		  }
		 */
		// order trails by the index of the trail
		int mostTrails = 0;
		for (Boid tBoid : flock.getBoids()) {
			Turtle t = ((TurtleBoid) tBoid).getTurtle();
			if (t.getTrailIndex() > mostTrails) mostTrails = t.getTrailIndex();
		}
		for (int i = 0; i < mostTrails + 1; i++) {
			for (Boid tBoid : flock.getBoids()) {
				Turtle t = ((TurtleBoid) tBoid).getTurtle();
				if (!t.isEmpty() && t.size() > i) {
					t.get(i).draw(pg);
				}
			}
		}
		for (Boid tBoid : flock.getBoids()) {
			Turtle t = ((TurtleBoid) tBoid).getTurtle();
			if (null != t.getCurrentTrail()) {
				t.getCurrentTrail().draw(pg);
			}
		}		  
		pg.endDraw();
		// println("offscreen done");
	}
	
	
	/**
	 * Creates a blobby Bezier curve shape at the given location.
	 * @param x   x coordinate
	 * @param y   y coordinate
	 * @return    a net.paulhertz.aifile.BezShape object centered on x and y
	 */
	public BezShape makeBlob(float x, float y) {
		// prepare to create a random blob
		int sectors = rando.randomInRange(4, 13);
		float rad = (float) rando.gauss(5.0, 0.25);
		float sectorVariance;
		do {
			sectorVariance = ((float) rando.quickGauss(0.1, 0.001));
		} while (sectorVariance < 0);
		float curveVariance = abs((float) rando.quickGauss(0.0, 0.001));
		float radiusVariance = abs((float) rando.quickGauss(0.0, 0.005));
		float angle = PApplet.radians(rando.randomInRange(0, 90));
		int[] channelValues = {55, 89, 123, 144, 199, 233};
		int farb = Palette.randColor(channelValues);
		BezShape shape = createBlob(sectors, rad, sectorVariance, curveVariance, radiusVariance, angle, x, y, farb);
		return shape;
	}
	
	float kappa = 0.5522847498f;
	float kk = kappa;
	/**
	 * creates a blobby Bezier shape
	 * sectorVariance of 0 sets all sectors equal, value > 1 may result in negative sector angles
	 * curveVariance of 0 sets all curvatures to approximate a circle 
	 * radiusVariance of 0 sets all radial distances equal (to radius variable)
	 * for example, createBlob(5, radius, 0.1, 0.0, 0.0) will draw circles with 5 slightly irregular sectors
	 * @param sectors          number of sectors
	 * @param rad              radius
	 * @param sectorVariance   variance in angular measurement of sectors (positive float)
	 * @param curveVariance    variance in amount of curvature (positive float)
	 * @param radiusVariance   variance in length of radius at each Bezier anchor point (positive float)
	 * @param rot              radians to rotate sectors
	 * @param dx               x-coordinate of center point (used for transforms)
	 * @param dy               y-coordinate of center point (used for transforms)
	 * @param farb             fill color for blob
	 */
	public BezShape createBlob(int sectors, float rad, float sectorVariance, float curveVariance, 
			float radiusVariance, float rot, float dx, float dy, int farb) {
		float[] sects = new float[sectors];
		float sum = 0;
		// fill the sects array with a series of increasing numbers at a nearly uniform 
		// distance from each other, with a variability determined by sectorVariance
		for (int i = 0; i < sectors; i++) {
			float num = (float) rando.quickGauss(2, sectorVariance);
			sum = sum + num;
			sects[i] = num;
			// println("sects[" + i + "] = " + num);
		}
		// println("sum = " + sum + "\n");
		// scale the sects array to fill a range of 2 * pi, its values  will determine
		// the angles between anchor points with respect to the center point dx, dy
		sects = reapportion(sects, PApplet.TWO_PI);
		// calculate the factor k that determines the distance between 
		// a Bezier anchor point and its associated control points
		float k = (4 * kk)/PApplet.TWO_PI;
		float d;
		// local variables: starting point, control point 1, control point 2, anchor point
		Point2D.Float pt, cp1, cp2, ap1;
		// cumulative rotation from the starting point
		float theta = 0;
		float kfac = (float) rando.quickGauss(1, curveVariance);
		float rfac = (float) rando.quickGauss(1, radiusVariance);
		float r = rad * rfac;
		float r0 = r;
		// starting point
		pt = GeomUtils.rotateCoor(r0, 0, rot);
		// an array for the points, one to start with, then three more for each Bezier vertex
		ArrayList<Point2D.Float> bezPoints = new ArrayList<Point2D.Float>();
		// translate starting point by dx, dy and add it to the array 
		bezPoints.add(GeomUtils.translateCoor(pt, dx, dy));
		for (int i = 0; i < sectors; i++) {
			kfac = (float) rando.quickGauss(1, curveVariance);
			// calculate distance between anchor point and control points
			d = sects[i] * k * r * kfac;
			// first control point
			cp1 = GeomUtils.rotateCoor(r, d, theta + rot);
			bezPoints.add(GeomUtils.translateCoor(cp1, dx, dy));
			theta += sects[i];
			if (i != sectors - 1) {
				rfac = (float) rando.quickGauss(1, radiusVariance);
				r = rad * rfac;
			} 
			else {
				r = r0;
			}
			// second control point
			cp2 = GeomUtils.rotateCoor(r, -d, theta + rot);
			bezPoints.add(GeomUtils.translateCoor(cp2, dx, dy));
			// anchor point
			ap1 = GeomUtils.rotateCoor(r, 0, theta + rot);
			bezPoints.add(GeomUtils.translateCoor(ap1, dx, dy));
		}
		// we have the points, now make a Bezier shape
		pt = bezPoints.get(0);
		BezShape bezo = new BezShape(this, pt.x, pt.y);
		bezo.setCenter(dx, dy);
		bezo.setFillColor(farb);
		bezo.setNoStroke();
		for (int i = 1; i < bezPoints.size(); i += 3) {
			cp1 = bezPoints.get(i);
			cp2 = bezPoints.get(i + 1);
			ap1 = bezPoints.get(i + 2);
			bezo.append(cp1.x, cp1.y, cp2.x, cp2.y, ap1.x, ap1.y);
		}
		return bezo;
	}

	/**
	 * @param series   an array of floats, all > 1.0
	 * @param range    the desired scaled sum of the values in series
	 * @return         the array with its values scaled so they sum to range
	 */
	public float[] reapportion(float[] series, float range) {
		float sum = 0;
		for (int i = 0; i < series.length; i++) {
			sum += series[i];
		}
		float d = range/sum;
		for (int i = 0; i < series.length; i++) {
			series[i] *= d;
		}
		sum = 0;
		for (int i = 0; i < series.length; i++) {
			float num = series[i];
			sum += num;
			// println("sects[" + i + "] = " + num);
		}
		// println("sum = " + sum + "\n");
		return series;
	}


	/************* Save to Illustrator File *************/

	/**
	 * @return   a string in the format yymmdd_hhmmss
	 */
	public String getTimestamp() {
		return nf(year(),2).substring(2, 4) +  nf(month(),2) + nf(day(),2) +"_"+ nf(hour(),2) + nf(minute(),2) + nf(second(),2);
	}

	
	// TODO export variations
	
	/**
	 * Saves trails (BezShape instances) associated with each boid to 
	 * an Adobe Illustrator file.
	 * @param filename   name of the file to save.
	 */
	public void saveAI(String filename) {
		PrintWriter pw = createWriter(filename +"_"+ fileCount++ +".ai");
		DocumentComponent document = new DocumentComponent(this, "TurtleBoid Trails");
		document.setVerbose(false);
		document.setCreator("Ignotus");
		document.setOrg("IgnoStudio");
		document.setWidth(width);
		document.setHeight(height);
		BezShape bgRect = BezRectangle.makeLeftTopWidthHeight(this, 0, 0, width, height);
		bgRect.setNoStroke();
		bgRect.setFillColor(color(254, 251, 246));
		LayerComponent bgLayer = new LayerComponent(this, "background");
		bgLayer.add(bgRect);
		bgLayer.setLocked(true);
		// begin adding components to boidsLayer
		LayerComponent boidsLayer = new LayerComponent(this, "boids");
/* 		
    // The layering used here puts trails from each boid into its own group, not the same as the display.
		// The getTrails() method returns a turtle's trails, saved and current, bundled into a GroupComponent.
		// The GroupComponent can be added to a DocumentComponent, LayerComponent, or another GroupComponent
		for (Boid tBoid : flock.getBoids()) {
			Turtle t = ((TurtleBoid) tBoid).getTurtle();
			boidsLayer.add(t.getTrails());
		}
*/		
		// here we send trails to groups determined by the index of the trail, more like what happens in the display
		int mostTrails = 0;
		for (Boid tBoid : flock.getBoids()) {
			Turtle t = ((TurtleBoid) tBoid).getTurtle();
			if (t.getTrailIndex() > mostTrails) mostTrails = t.getTrailIndex();
		}
		println("Number of groups = "+ mostTrails);
		for (int i = 0; i < mostTrails + 1; i++) {
			GroupComponent g = new GroupComponent(this);
			for (Boid tBoid : flock.getBoids()) {
				Turtle t = ((TurtleBoid) tBoid).getTurtle();
				if (!t.isEmpty() && t.size() > i) {
					BezShape bez = t.get(i);
					g.add(bez);
					// white half-width copy on top
					/**/
					BezShape bezCopy = bez.clone();
					bezCopy.setWeight(bezCopy.weight() * 0.5f);
					bezCopy.setStrokeColor(color(255));
					g.add(bezCopy);
					/**/
				}
			}
			boidsLayer.add(g);
		}
		GroupComponent g = new GroupComponent(this);
		for (Boid tBoid : flock.getBoids()) {
			Turtle t = ((TurtleBoid) tBoid).getTurtle();
			if (null != t.getCurrentTrail()) {
				g.add(t.getCurrentTrail());
			}
		}
		boidsLayer.add(g);
		// end adding components to boidsLayer
		document.add(bgLayer);
		document.add(boidsLayer);
		document.writeWithAITransform(pw);
		println("-- saved AI document "+ filename);
	}
	
	/**
	 * Saves trails (BezShape instances) associated with each boid to 
	 * an Adobe Illustrator file.
	 * @param filename   name of the file to save.
	 */
	public void saveListAI(String filename) {
		filename = filePath +"/"+ filename +"_"+ getTimestamp() +"_"+ fileCount++ + ".ai";
		PrintWriter pw = createWriter(filename);
		println("-- Saving "+ filename);
		DocumentComponent document = new DocumentComponent(this, "TurtleBoid Trails");
		document.setVerbose(false);
		document.setCreator("Ignotus");
		document.setOrg("IgnoStudio");
		document.setWidth(width);
		document.setHeight(height);
		BezShape bgRect = BezRectangle.makeLeftTopWidthHeight(this, 0, 0, width, height);
		bgRect.setNoStroke();
		bgRect.setFillColor(color(254, 251, 246));
		LayerComponent bgLayer = new LayerComponent(this, "background");
		bgLayer.add(bgRect);
		bgLayer.setLocked(true);
		// begin adding components to boidsLayer
		LayerComponent boidsLayer = new LayerComponent(this, "boids");
		for (GroupComponent g : gList) {
			boidsLayer.add(g);
		}
		// end adding components to boidsLayer
		document.add(bgLayer);
		document.add(boidsLayer);
		document.writeWithAITransform(pw);
		println("-- saved AI document "+ filename);
	}

	
	/************* Control Panel *************/

	// TODO 
	/**
	 * Sets up the control panel.
	 */
	public void loadPanel() {
		int panelBack = this.color(123, 123, 144, 255);
		int settingsWidth = 270;
		int panelHeight = 280;
		int startPos = 20;
		int yPos = startPos + step + step + step + 4;
		int step = 18;
		int widgetH = 14;
		int labelW = 144;
		settings = controlP5.addGroup("Global Settings", 4, 20, settingsWidth);
		settings.setBackgroundColor(panelBack);
		settings.setBackgroundHeight(panelHeight);
		settings.setBarHeight(widgetH);
		settings.setMoveable(true);
		// add widgets
		// separation
		Numberbox n1 = controlP5.addNumberbox("setSeparation", sep, 8, yPos, 100, widgetH);
		n1.setGroup(settings);
		n1.setMultiplier(1.0f);
		n1.setMin(1.0f);
		n1.setMax((min(width, height)));
		n1.setCaptionLabel("");
		// label for separation number box
		Textlabel l1 = controlP5.addTextlabel("separationLabel", "SEPARATION ", 112, yPos + 4);
		l1.setGroup(settings);
		// alignment
		yPos += step;
		Numberbox n2 = controlP5.addNumberbox("setAlignment", align, 8, yPos, 100, widgetH);
		n2.setGroup(settings);
		n2.setMultiplier(1.0f);
		n2.setMin(1.0f);
		n2.setMax((max(width, height)));
		n2.setCaptionLabel("");
		// label for alignment number box
		Textlabel l2 = controlP5.addTextlabel("alignmentLabel", "ALIGNMENT ", 112, yPos + 4);
		l2.setGroup(settings);
		// coherence
		yPos += step;
		Numberbox n3 = controlP5.addNumberbox("setCohesion", coh, 8, yPos, 100, widgetH);
		n3.setGroup(settings);
		n3.setMultiplier(1.0f);
		n3.setMin(1.0f);
		n3.setMax((max(width, height)));
		n3.setCaptionLabel("");
		// label for coherence number box
		Textlabel l3 = controlP5.addTextlabel("cohesionLabel", "COHESION ", 112, yPos + 4);
		l3.setGroup(settings);
		// use factors check box
		yPos += step + 4;
		CheckBox ch1 = controlP5.addCheckBox("setUseFactors", 8, yPos);
		ch1.addItem("Use Scaling Factors", 1);
		ch1.setGroup(settings);
		ch1.activate(1);
		// separation ration
		yPos += step - 2;
		Numberbox n0 = controlP5.addNumberbox("setSepFac", sepFac, 8, yPos, 100, widgetH);
		n0.setGroup(settings);
		n0.setMultiplier(0.25f);
		n0.setMin(0.25f);
		n0.setMax(800.0f);
		n0.setCaptionLabel("");
		// label for alignment factor number box
		Textlabel l0 = controlP5.addTextlabel("alignSepLabel", "SEPARATION RATIO", 112, yPos + 4);
		l0.setWidth(labelW);
		l0.setGroup(settings);
		// alignment factor
		yPos += step - 2;
		Numberbox n4 = controlP5.addNumberbox("setAlignFac", alignFac, 8, yPos, 100, widgetH);
		n4.setGroup(settings);
		n4.setMultiplier(0.25f);
		n4.setMin(0.25f);
		n4.setMax(800.0f);
		n4.setCaptionLabel("");
		// label for alignment factor number box
		Textlabel l4 = controlP5.addTextlabel("alignFacLabel", "ALIGNMENT RATIO", 112, yPos + 4);
		l4.setWidth(labelW);
		l4.setGroup(settings);
		// cohesion factor
		yPos += step;
		Numberbox n5 = controlP5.addNumberbox("setCohFac", cohFac, 8, yPos, 100, widgetH);
		n5.setGroup(settings);
		n5.setMultiplier(0.25f);
		n5.setMin(0.25f);
		n5.setMax(800.0f);
		n5.setCaptionLabel("");
		// label for alignment factor number box
		Textlabel l5 = controlP5.addTextlabel("cohFacLabel", "COHESION RATIO", 112, yPos + 4);
		l5.setWidth(labelW);
		l5.setGroup(settings);
		// add or subtract boids
		yPos += step + 4;
		Numberbox n6 = controlP5.addNumberbox("setNumberOfBoids", numberOfBoids, 8, yPos, 100, widgetH);
		n6.setGroup(settings);
		n6.setMultiplier(1.0f);
		n6.setMin(1.0f);
		n6.setMax(233.0f);
		n6.setCaptionLabel("");
		// label for number of boids number box
		Textlabel l6 = controlP5.addTextlabel("numberOfBoidsLabel", "NUMBER OF BOIDS", 112, yPos + 4);
		l6.setWidth(labelW);
		l6.setGroup(settings);
		yPos += step;
		Button b1 = controlP5.addButton("addBoids", 0).setPosition(8, yPos).setSize(76, widgetH);
		b1.setGroup(settings);
		b1.setCaptionLabel("Add Boids");
		Button b2 = controlP5.addButton("subtractBoids", 0).setPosition(settingsWidth/3 + 4, yPos).setSize(76, widgetH);
		b2.setGroup(settings);
		b2.setCaptionLabel("Subtract Boids");
		Button b3 = controlP5.addButton("newBoids", 0).setPosition(2 * settingsWidth/3, yPos).setSize(76, widgetH);
		b3.setGroup(settings);
		b3.setCaptionLabel("New Boids");
		yPos += step;
		Textlabel l7 = controlP5.addTextlabel("totalBoidsLabel", "Total boids: " + totalBoids, 8, yPos + 4);
		l7.setGroup(settings);
		yPos += step;
		Button b4 = controlP5.addButton("erase", 0).setPosition(8, yPos).setSize(76, widgetH);
		b4.setGroup(settings);
		b4.setCaptionLabel("Erase");
		Button b5 = controlP5.addButton("pauseLoop", 0).setPosition(settingsWidth/3 + 4, yPos).setSize(76, widgetH);
		b5.setGroup(settings);
		b5.setCaptionLabel("Pause");
		Button b6 = controlP5.addButton("runLoop", 0).setPosition(2 * settingsWidth/3, yPos).setSize(76, widgetH);
		b6.setGroup(settings);
		b6.setCaptionLabel("Run");
		yPos += step + 4;
		Numberbox n8 = controlP5.addNumberbox("setStrokeWeight", weight, 8, yPos, 100, widgetH);
		n8.setGroup(settings);
		n8.setMultiplier(0.25f);
		n8.setMin(0.25f);
		n8.setMax(48.0f);
		n8.setCaptionLabel("");
		// label for number of boids number box
		Textlabel l8 = controlP5.addTextlabel("strokeWeightLabel", "STROKE WEIGHT", 112, yPos + 4);
		l8.setWidth(labelW);
		l8.setGroup(settings);
		int menuH = widgetH + 2;
		yPos = yPos + step + step;
		// drop down list for boid states
		DropdownList dd2 = controlP5.addDropdownList("statelist", 8, yPos-12, 200, widgetH + 2);
		menuH = widgetH + 2;
		dd2.setBarHeight(menuH);
		dd2.setItemHeight(menuH);
		dd2.setHeight(boidStateList.size() * menuH + menuH + 2);
		for (int i = 0; i < boidStateList.size(); i++) {
			dd2.addItem(boidStateList.get(i).name, i);
		}
		dd2.setOpen(false);
		dd2.setGroup(settings);
		// drop down list for video devices
		DropdownList dd1 = controlP5.addDropdownList("devicelist", 8, startPos-20, 240, widgetH + 2);
		dd1.setBarHeight(menuH);
		dd1.setItemHeight(menuH);
		String[] devices = getVideoDevices();
		dd1.setHeight(devices.length * menuH + menuH + 2);
		for (int i = 0; i < devices.length; i++) {
			dd1.addItem(devices[i], i);
		}
		dd1.setOpen(false);
		dd1.setGroup(settings);
		// finish
		settings.moveTo("global");
		settingsTab = controlP5.getTab("default");
		settingsTab.activateEvent(true);
		settingsTab.setLabel("Flock Settings");
		settingsTab.setId(1);
	}
	
	/**
	 * Called by control panel number box, sets separation for boids. 
	 * @param value  the new separation value for each boid. If useFactors is true, 
	 * align and coh will be set using alignFac and cohFac and propagated to each boid.
	 */
	public void setSeparation(float value) {
		if (value >= PApplet.max(width, height)) return;
		if (value < 1) return;
		sep = value;
		if (useFactors) {
			align = sep * (alignFac/sepFac);
			coh = sep * (cohFac/sepFac);
			Numberbox n2 = (Numberbox) controlP5.getController("setAlignment");
			Numberbox n3 = (Numberbox) controlP5.getController("setCohesion");
			n2.setValue(align);
			n3.setValue(coh);
		}
		adjustFlock();
	}
	
	/**
	 * Called by control panel number box, sets alignment for each boid.
	 * @param value
	 */
	public void setAlignment(float value) {
		align = value;
		adjustFlock();
	}
	
	/**
	 * Called by control panel number box, sets cohesion for each boid.
	 * @param value
	 */
	public void setCohesion(float value) {
		coh = value;
		adjustFlock();
	}
	
	/**
	 * Called by control panel number box
	 * @param value   new value for sepFac
	 */
	public void setSepFac(float value) {
		sepFac = value;
	}
	
	/**
	 * Called by control panel number box, sets alignFac.
	 * @param value   new value for alignFac.
	 */
	public void setAlignFac(float value) {
		alignFac = value;
		if (useFactors) {
			align = sep * alignFac/sepFac;
			adjustFlock();
		}
	}
	
	/**
	 * Called by control panel number box, sets cohFac.
	 * @param value   new value for cohFac.
	 */
	public void setCohFac(float value) {
		cohFac = value;
		if (useFactors) {
			coh = sep * cohFac/sepFac;
			adjustFlock();
		}
	}
	
	/**
	 * TODO be sure controls correctly reflect the initial settings
	 * Called by control panel checkbox "setUseFactors," locks or unlocks 
	 * alignment and cohesion number boxes.
	 * @param evt
	 */
	public void controlEvent(ControlEvent evt) {
		if (evt.isGroup() && evt.getGroup().getName().equals("setUseFactors")) {
			int n = (int) evt.getGroup().getArrayValue()[0];
			useFactors = (n > 0);
			Numberbox n2 = (Numberbox) controlP5.getController("setAlignment");
			Numberbox n3 = (Numberbox) controlP5.getController("setCohesion");
			if (useFactors) {
				n2.lock();
				n3.lock();
			}
			else {
				n2.unlock();
				n3.unlock();
			}
		}
		else if (evt.isController() && evt.getController().getName().equals("devicelist")) {
			int index = (int) evt.getController().getValue();
			String[] devices = getVideoDevices();
			// name=Built-in iSight,size=320x240,fps=30
			String device = devices[index];
			//println("selected device is "+ device +", index = "+ index);
			String[] info = device.split(",");
			//for (int i = 0; i < info.length; i++) {
			//	println(info[i]);
			//}
			String deviceName = info[0].substring(info[0].indexOf("=") + 1);
			println("deviceName = "+ deviceName);
			String dimensions = info[1].substring(info[1].indexOf("=") + 1);
			//println("dimensions = "+ dimensions);
			String[] coords = dimensions.split("x");
			int w = Integer.valueOf(coords[0]);
			int h = Integer.valueOf(coords[1]);
			println("width = "+ w);
			println("height = "+ h);
			String fps = info[2].substring(info[2].indexOf("=") + 1);
			int framerate = Integer.valueOf(fps);
			println("fps = "+ fps);
			int grid = w/40;
			println("grid = "+ grid);
			float predictionSeconds = 0.25f * 30.0f/framerate;
			setupVideo(w, h, framerate, grid, predictionSeconds, deviceName);		
		}
		else if (evt.isController() && evt.getController().getName().equals("statelist")) {
			selectedBoidState = (int) evt.getController().getValue();
			selectedBoidState = (selectedBoidState) % boidStateList.size();
			assignBoidState(selectedBoidState, 1f);
			println("selectedBoidState: "+ selectedBoidState);
			
		}
		// debugging
		/*
		if (evt.isGroup()) {
			println(evt.getGroup().getValue() +" from group "+ evt.getGroup() +"with name "+ evt.getGroup().getName());
		}
		else if (evt.isController()) {
			println(evt.getController().getValue() +" from controller "+ evt.getController() +"with name "+ evt.getController().getName());
		}
		*/
	}
	
	/**
	 * Called by control panel, sets number of boids to add or subtract.
	 * @param value   number of boids to add or subtract
	 */
	public void setNumberOfBoids(float value) {
		numberOfBoids = (int) value;
	}
	
	/**
	 * Adds numberOfBoids boids to flock.
	 */
	public void addBoids() {
		for (int i = 0; i < numberOfBoids; i++) {
			addOneBoid(random(0, width), random(0, height));
			totalBoids++;
		}
		Textlabel l7 = (Textlabel) controlP5.getController("totalBoidsLabel");
		l7.setValue("Total boids: " + totalBoids);
	}
	
	/**
	 * Subtracts numberOfBoids boids from flock.
	 */
	public void subtractBoids() {
		ArrayList<Boid> boids = flock.getBoids();
		for (int i = 0; i < numberOfBoids; i++) {
			totalBoids -= 1;
			if (totalBoids < 2) return;
			boids.remove(0);
			// println("removed a boid");
		}
		Textlabel l7 = (Textlabel) controlP5.getController("totalBoidsLabel");
		l7.setValue("Total boids: " + totalBoids);
	}
	
	/**
	 * Creates a new flock.
	 */
	public void newBoids() {
		ArrayList<Boid> boids = flock.getBoids();
		boids.clear();
//		for (int i = 0; i < totalBoids; i++) {
//			addOneBoid(random(0, width), random(0, height));
//		}		
		initBoids();
		Textlabel l7 = (Textlabel) controlP5.getController("totalBoidsLabel");
		l7.setValue("Total boids: " + totalBoids);
	}
	
	/**
	 * Erases all stored trails in boids (TurtleBoids), clears offscreen buffer.
	 */
	public void erase() {
		for (Boid tBoid : flock.getBoids()) {
			((TurtleBoid) tBoid).getTurtle().clear();
		}
		bgErase();
	}
	
	public void bgErase() {
		drawOffscreen(pg);	
	}
	// TODO fix drawOffscreen to use opacity
	public void bgErase(int opacity) {
		drawOffscreen(pg);	
	}
		
	/**
	 * Pauses draw method.
	 */
	public void pauseLoop() {
		isPaused = true;
	}
	
	/**
	 * Starts draw method.
	 */
	public void runLoop() {
		isPaused = false;
	}
	
	/**
	 * @param newWeight  new stroke weight to set for each boid
	 */
	public void setStrokeWeight(float newWeight) {
		weight = newWeight;
		for (Boid tBoid : flock.getBoids()) {
			Turtle t = ((TurtleBoid) tBoid).getTurtle();
			t.setWeight(weight);
			float w = (float) rando.gauss(newWeight, 0.01);
			if (w > 0) t.setWeight(w);
		}
	}
	
	/**
	 * Sets the separation, alignment and cohesion parameters for each boid.
	 */
	public void adjustFlock() {
		for (Boid tBoid : flock.getBoids()) {
			float separation = (float) rando.gauss(sep, 0.005);
			if (separation > 0) tBoid.setSeparationDistance(separation);
			else tBoid.setSeparationDistance(sep);
			float alignment = (float) rando.gauss(align, 0.005);
			if (alignment > 0) tBoid.setAlignmentDistance(alignment);
			else tBoid.setAlignmentDistance(align);
			float cohesion = (float) rando.gauss(coh, 0.005);
			if (cohesion > 0) tBoid.setCohesionDistance(cohesion);
			else tBoid.setCohesionDistance(coh);
		}
	}

	/**
	 * @param colorShape   any object that implements ColorableINF
	 * @param targHue      a target hue in the range 0..360
	 * @param variance     variance around the target hue
	 */
	public void setShapeStroke(ColorableINF colorShape, float targHue, float variance) {
		// set mode to Hue, Saturation, Brightness
		colorMode(HSB, 360, 100, 100);
		double hue = rando.gauss(targHue, variance);
		if (hue > 360) hue -= 360;
		if (hue < 0) hue += 360;
		hue = Math.round(hue);
		int newColor = this.color((int)hue, random(55, 89), random(68, 97));
		// set mode back to Red, Green, Blue
		colorMode(RGB, 255, 255, 255);
		colorShape.setStrokeColor(newColor);
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
		PImage newImage = createImage(w, h, ARGB);
		newImage.loadPixels();
		arrayCopy(image.pixels, newImage.pixels);
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
	public static int composeColor(int[] comp) {
		return 255 << 24 | comp[0] << 16 | comp[1] << 8 | comp[2];
	}

	
	/************* Responder Class *************/

	/**
	 * Responder is a callback class called by TurtleBoid.
	 */
	class Responder implements BoidCallbackINF {
		/* (non-Javadoc)
		 * @see com.ignofactory.steering.BoidCallbackINF#callback(com.ignofactory.steering.Boid)
		 */
		@Override
		public void callback(Boid tBoid) {
			/* if (tBoid instanceof TurtleBoid) println("it's a turtle boid"); */
			// NOTE: a penUp() was issued before we arrived here
			// followed by a conditional pen down, if the turtle was drawing
			Turtle t = ((TurtleBoid)tBoid).getTurtle();
			stepColor(tBoid, t, 15);
			// add the latest trail to the offscreen graphics pg
			if (!t.isEmpty()) {
				BezShape bez = t.getTurtleTrails().get(t.getTrailIndex());
				pg.beginDraw();
				bez.draw(pg);
				pg.endDraw();
			}
			// pen goes down if start on edge is set
			if (beginOnEdge) t.penDown();
			// stopOnEdge overrides beginOnEdge
			if (stopOnEdge) t.penUp();
			/*	println("call back from TurtleBoid id "+ ((TurtleBoid)tBoid).id +" with hue "+ targHue); */
		}
		
		public void stepColor(Boid tBoid, Turtle t, float stepSize) {
			int tbStrokeColor = t.strokeColor();
			colorMode(HSB, 360, 100, 100);
			float targHue = hue(tbStrokeColor) + 15;
			colorMode(RGB, 255, 255, 255);
			setShapeStroke(((TurtleBoid)tBoid), targHue, 30);
		}
	}
	
	
	/************* VideoResponder Class *************/

	boolean skipAction = false;
	class VideoResponder implements VideoCallbackINF {
		float actionThreshold = 0.5f;
		float sepMax = 233;
		float sepMin = 3.0f;
		int evtTimer1, evtTimer2, evtTimer3, evtTimer4 = millis();
		int evtDebounce = 240;
		int inset = 120; 
		PVector btn1 = new PVector(inset, inset);                      // top left
		PVector btn2 = new PVector(inset, height - inset);             // bottom left
		PVector btn3 = new PVector(width - inset, height - inset);     // bottom right
		PVector btn4 = new PVector(width - inset, inset);              // top right
		int color1 = color(233, 89, 55);              // red (top left button)
		int color2 = color(55, 233, 144);             // green (bottom left button)
		int color3 = color(220, 165, 55);             // yellow (bottom right button)
		int color4 = color(55, 21, 233);              // blue (top right button)
		int inactive = color(216, 216, 216, 192);     // gray (inactive state)
		float flowMin = 999999999f;
		float flowMax = 0;
		float minTrigger = 1.0f;
		float maxTrigger = 5.0f;
		boolean isMinState = false;
		boolean isMaxState = false;

		@Override
		public void videoCallback(Capture video) {
			// get the video image, give it an alpha channel and draw it on our display
			background(pg);
			PImage img = loadImageAlpha(video.get(), 80);
			// we love pixels, can't call noSMooth outside setup() in Processing 3.x
			// noSmooth();
			image(img, 0, 0, width, height);
			smooth();
		}

		@Override
		public void vectorCallback(Capture video) {
			drawVectorLines();
		}
		
		@Override
		public void actionCallback(Capture video) {
			if (skipAction) return;
			float flowRate = optical.getTotalFlowSquareMagAv();
			if (flowRate < minTrigger && !isMinState) {
				erase();
				isMinState = true;
				isMaxState = false;
				println("-------- min state --------");
			} 
			else if (flowRate > maxTrigger && !isMaxState) {
				decode('a');
				isMaxState = true;
				isMinState = false;
				println("-------- MAX state --------");
			}
			/*
			if (flowRate < flowMin) {
				flowMin = flowRate;
				println("flowMin = "+ flowMin);
			}
			else if (flowRate > flowMax) {
				flowMax = flowRate;
				println("flowMax = "+ flowMax);
			}
			*/
			/* */
			// trigger call to setSeparation() by setting the number box, avoid recursion
			Numberbox n1 = (Numberbox) controlP5.getController("setSeparation");
			float mag1 = optical.getFlow(btn1).mag();
			float mag2 = optical.getFlow(btn2).mag();
			float mag3 = optical.getFlow(btn3).mag();
			float mag4 = optical.getFlow(btn4).mag();
			int now = millis();
			// top left
			if (mag1 > actionThreshold) {
				markGrid((int)btn1.x, (int)btn1.y, color1);
				if (now - evtTimer1 > evtDebounce) {
					// perform an action and reset the timer
					if (totalBoids < maxBoids) addBoids();
					evtTimer1 = now;					
				} 
			}
			/*
			else {
				markGrid((int)btn1.x, (int)btn1.y, inactive);				
			}
			// bottom left
			if (mag2 > actionThreshold) {
				markGrid((int)btn2.x, (int)btn2.y, color2);
				if (now - evtTimer2 > evtDebounce) {
					// perform an action and reset the timer
					bgErase(64);
					evtTimer2 = now;					
				}
			}
			else {
				markGrid((int)btn2.x, (int)btn2.y, inactive);
			}
			// bottom right
			if (mag3 > actionThreshold) {
				markGrid((int)btn3.x, (int)btn3.y, color3);
				if (now - evtTimer3 > evtDebounce) {
					// perform an action and reset the timer
					decode('a');
					evtTimer3 = now;					
				}
			}
			else {
				markGrid((int)btn2.x, (int)btn2.y, inactive);
			}
			*/
			// top right
			if (mag4 > actionThreshold) {
				markGrid((int)btn4.x, (int)btn4.y, color4);		
				if (now - evtTimer4 > evtDebounce) {
					// perform an action and reset the timer
					if (totalBoids > minBoids) subtractBoids();
					evtTimer4 = now;					
				}
			}
			else {
				markGrid((int)btn4.x, (int)btn4.y, inactive);								
			}
			adjustFlock();
			/* */
		}
		
		public void markGrid(int x, int y, int fillColor) {
			// find the grid
			ellipseMode(PApplet.CENTER);
			noStroke();
			fill(fillColor);
			ellipse(x, y, 20, 20);
		}
		
	}
	
	// still some problems working with boids -- flickering
	public void drawVectorLines(PGraphics offscreen) {
		offscreen.beginDraw();
		offscreen.background(192, 192, 192, 127);
		offscreen.pushStyle();
		offscreen.stroke(color(55, 55, 89, 255));
		offscreen.strokeWeight(1);
		float[] vx = optical.getVxcoords();
		float[] vy = optical.getVycoords();
		PVector[] vec = optical.getFlowList();
		if (null == vec || 0 >= vec.length || null == vec[0]) {
			println("vec array is empty");
			return;
		}
		float flowScale = optical.getFlowScale();
		// TODO scale flow lines when drawing window has different proportions from video window
		int rowCount = optical.getGw();
		float displayRatio = width/(float) height;
		float videoRatio = videoWidth/(float) videoHeight;
		float vs = videoRatio/displayRatio;
		for (int i = 0; i < vec.length; i++) {
			float u = vec[i].x;
			float v = vec[i].y;
			float x0 = vx[i];
			float y0 = vy[i];
			/*
			if (i % rowCount == 0 || i % rowCount == rowCount - 1
					|| i < rowCount || i > vec.length - rowCount) {
				offscreen.stroke(color(233, 0, 89, 255));
				offscreen.fill(color(233, 0, 89, 255));
				// offscreen.ellipse(x0 * hs, y0 * vs, 5, 5);
				offscreen.ellipse(x0, y0 * vs, 5, 5);
			}
			else {
				offscreen.stroke(color(55, 55, 89, 255));
			}
			*/
			offscreen.stroke(color(55, 55, 89, 255));
			float a = PApplet.sqrt(u * u + v * v);
			if(a >= 2.0) {
				// offscreen.line(x0, y0, x0 + u * flowScale, y0 + v * flowScale);
				offscreen.line(x0, y0 * vs, x0 + u * flowScale, y0 * vs + v * flowScale);
				// offscreen.line(x0, y0, x0 + u * hs, y0 + v * vs);
				// offscreen.line(x0 * hs - g1, y0 * vs, x0 * hs + u * hs - g1, y0 * vs + v * vs);
			}
		}
		offscreen.popStyle();
		offscreen.endDraw();
	}

	
	public void drawVectorLines() {
		pushStyle();
		stroke(color(55, 55, 89, 127));
		strokeWeight(1);
		float[] vx = optical.getVxcoords();
		float[] vy = optical.getVycoords();
		PVector[] vec = optical.getFlowList();
		float flowScale = optical.getFlowScale();
		// TODO scale flow lines when drawing window has different proportions from video window
		int rowCount = optical.getGw();
		float displayRatio = width/(float) height;
		float videoRatio = videoWidth/(float) videoHeight;
		float vs = videoRatio/displayRatio;
		for (int i = 0; i < vec.length; i++) {
			float u = vec[i].x;
			float v = vec[i].y;
			float x0 = vx[i];
			float y0 = vy[i];
			float a = PApplet.sqrt(u * u + v * v);
			if(a >= 2.0) {
				line(x0, y0 * vs, x0 + u * flowScale, y0 * vs + v * flowScale * vs);
			}
		}
		popStyle();
	}

	public void drawVectorEllipses() {
		pushStyle();
		noStroke();
		fill(color(233, 55, 89, 127));
		strokeWeight(1);
		float[] vx = optical.getVxcoords();
		float[] vy = optical.getVycoords();
		PVector[] vec = optical.getFlowList();
		float flowScale = optical.getFlowScale();
		for (int i = 0; i < vec.length; i++) {
			float u = vec[i].x;
			float v = vec[i].y;
			float x0 = vx[i];
			float y0 = vy[i];
			float a = PApplet.sqrt(u * u + v * v);
			if(a >= 2.0) {
				ellipse(x0, y0, a, a);
			}
		}
		popStyle();
	}

	
	// state-tracking class for boids
	class BoidState {
		public int sep;
		public int align;
		public int coh;
		public String name;
		
		public BoidState(int _sep, int _align, int _coh, String _name) {
			this.sep = _sep;
			this.align = _align;
			this.coh = _coh;
			this.name = _name;
		}		
	}
	
}
	

