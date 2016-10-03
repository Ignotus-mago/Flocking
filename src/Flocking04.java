import java.awt.Container;
import java.awt.Frame;
import java.awt.geom.Point2D;
import java.io.PrintWriter;
import java.util.ArrayList;

import net.paulhertz.geom.GeomUtils;
import net.paulhertz.util.RandUtil;

import com.ignofactory.steering.*;
import net.paulhertz.aifile.*;

import controlP5.*;

import processing.core.*;
import processing.video.*;


/**
 * @author paulhz
 *
 * Fifth version of flocking application, calls OpticalFlowMaker for video flow tracking.
 * Based on Flocking, by Daniel Shiffman <http://www.shiffman.net>, in The Nature of Code, Spring 2009.
 * Demonstration of Craig Reynolds' "Flocking" behavior, See: http://www.red3d.com/cwr/.
 * Rules: Cohesion, Separation, Alignment
 * 
 * Requires IgnoCodeLib (see https://processing.org/reference/libraries/) and ControlP5 (http://www.sojamo.de/libraries/controlP5/).
 * 
 * First version to use video tracking. Flocking05 solves most of the problems still present in this version.
 * 
 * In both versions, you will probably need to tweak video for your particular setup. 
 * The call "optical = new OpticalFlowMaker(this, width, height, 20, 0.5f, "FaceTime HD Camera");" in setup 
 * should be changed for your particular camera model and display/video dimensions.
 * 
 */
public class Flocking04 extends PApplet {
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
	// Press spacebar to show or hide controls
	// Press 'd' to toggle drawing
	// Press 'r' to toggle immediate display of drawing
	// Press 'c' to shift color
	// Press 'x' to erase
	// Press 'p' to pause
	// Press 's' to save to AI 7.0 format file
	// Press 'v' to toggle visibility of boids
	// Press 'l' to toggle cohesion lines (best with few boids)
	// Press 't' to toggle topology from torus to plane
	// Press 'w' to toggle wind
	// Press '+' or '-' to increase or decrease boids' separation
	// Press UP or DOWN arrow keys to increase or decrease wind force
	// Press LEFT or RIGHT arrow keys to turn boids.
	// Press mouse to attract boids (when controls are hidden)
	// Drag mouse to push or pull boids (when controls are hidden)
	// Video-tracking/Optical flow controls:
	//    Press 'f' to show or hide flow lines
	//    Press 'i' to show or hide video image
	//    Make motion in the upper right corner to increase boid separation (green dot appears)
	//    Make motion in the upper left corner to decrease boid separation (red dot appears)
	// Press 'h' to show help message 
	
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
	boolean flockIsDisplaying;
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
	int totalBoids = 23;
	boolean isPaused = false;
	boolean isShowBoids = false;
	boolean isShowVideo = true;
	float avoidance;
	// wind parameters
	boolean isWindy = false;
	float t = 0; 
	float divisor = 128;
	float inc = PApplet.PI/divisor;
	float windspeed = 8.0f;
	float maxWind = 5.0f;
	float minWind = 0.01f;
	float windIncrement = 0.1f;
	float turbulenceIncrement = 0.001f;
	public static final float PHI = (1 + (float) Math.sqrt(5)) * 0.5f;
	public int fileCount = 1;
	public ArrayList<GroupComponent> gList;
	public int step = 0;
	boolean stopOnEdge = false;
	boolean beginOnEdge = false;
	boolean dashedLine = false;
	boolean neonEffect = false;
	enum BoidPlacement {GRID, CENTER, RANDOM, RANDOMCENTER;}
	BoidPlacement placement = BoidPlacement.RANDOM;
	ArrayList<BoidState> boidStateList;
	OpticalFlowMaker optical;
	PImage maskImage;
	Frame myFrame;
	boolean isAutoRun = false;
	int selectedBoidState = 0;
	
	/**
	 * @param args
	 */
	public static void main(String[] args) {
		PApplet.main(new String[] { "--present", "Flocking04" });
	}

	public void setup() {
		// for an 800 by 600 space
		size(1280, 720);
		smooth();
		rando = new RandUtil();
		pg = createGraphics(width, height);
		pg.beginDraw();
		pg.smooth();
		pg.background(255);
		pg.endDraw();
		responder = new Responder();
		flock = new Flock();
		// Add an initial set of boids into the system
		flockIsDrawing = true;
		flockIsDisplaying = true;
		totalBoids = 29;
		initBoidStateList();
		// assignBoidState(rando.randomInRange(0, boidStateList.size() - 1), 0.8f);
		sep = 21; align = 47; coh = 89;
		sepFac = sep; alignFac = align; cohFac = coh;
		useFactors = true;
		placement = BoidPlacement.values()[rando.randomInRange(0, BoidPlacement.values().length - 1)];
		println("placement = "+ placement.toString());
		// placement = BoidPlacement.CENTER;
		controlP5 = new ControlP5(this);
		loadPanel();
		controlP5.hide();
		printHelp();
		println("PHI = "+ PHI);
		gList = new ArrayList<GroupComponent>();
		//printDevices();
		// optical = new OpticalFlowMaker(this, width, height, 20, 0.5f, "IIDC FireWire Video");
		// optical = new OpticalFlowMaker(this, width, height, 20, 0.5f, "USB Video Class Video");
		// optical = new OpticalFlowMaker(this, width, height, 20, 0.5f);
		// optical = new OpticalFlowMaker(this, width, height, 20, 0.5f, Capture.list()[0]);
		// optical = new OpticalFlowMaker(this, width, height, 20, 0.5f, "Built-in iSight");
		optical = new OpticalFlowMaker(this, width, height, 20, 0.5f, "FaceTime HD Camera");
		optical.setFlowColor(color(233, 220, 199, 127));
		optical.setImageFlowColor(color(144, 110, 233, 127));
		videoResponder = new VideoResponder();
		optical.setResponder(videoResponder);
		initMask();
		// initBlueNoise();
		initBoids();
		// Processing initializes the frame and hands it to you in the "frame" field.
		// Eclipse does things differently. Use findFrame method to get the frame in Eclipse.
		/*
		myFrame = findFrame();
		myFrame.setResizable(true);
		myFrame.setSize(displayWidth, displayHeight);
		*/
	}
	
	public boolean sketchFullScreen() {
		return true;
	}
	
	/**
	 * @return   Frame where Processing draws, useful method in Eclipse
	 */
	public Frame findFrame() {
		Container f = this.getParent();
		while (!(f instanceof Frame) && f!=null)
			f = f.getParent();
		return (Frame) f;
	}

	public void getVideoDevice() {
		String[] devices = Capture.list();
	}
	
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
		boidStateList.add(new BoidState(89, 29, 76, "untitled 1"));
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
		// sep = 13; align = 47; coh = 34;
		boidStateList.add(new BoidState(13, 47, 34, "untitled 5"));
		// sep = 21; align = 47; coh = 55;
		boidStateList.add(new BoidState(21, 47, 55, "untitled 6"));
		// sep = 34; align = 55; coh = 89;
		boidStateList.add(new BoidState(34, 55, 89, "untitled 7"));
		// sep = 13; align = 108; coh = 123;
		boidStateList.add(new BoidState(13, 108, 123, "untitled 8"));
		// sep = 21; align = 108; coh = 123;
		boidStateList.add(new BoidState(21, 108, 123, "untitled 9"));
		// sep = 34; align = 108; coh = 123;
		boidStateList.add(new BoidState(34, 108, 123, "untitled 10"));
	}
	
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
		println("assigned sep = "+ sep +", align = "+ align +", coh = "+ coh + " -- "+ state.name);
	}
	
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
	
	public void initBoids() {
		int[] hues = {0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330};
		targetHue = rando.randomElement(hues);
		// test a placement
		// placement = BoidPlacement.GRID;
		switch (placement) {
		case RANDOM: {
			for (int i = 0; i < totalBoids; i++) {
				addOneBoid(random(0, width), random(0, height));
			}
			break;
		}
		case CENTER: {
			for (int i = 0; i < totalBoids; i++) {
				addOneBoid(width/2, height/2);
			}
			break;
		}
		case RANDOMCENTER: {
			int xctr = (int) rando.gauss(width/2, 2 * width);
			int yctr = (int) rando.gauss(height/2, 2 * height);
			println("xctr = "+ xctr +", yctr = "+ yctr);
			for (int i = 0; i < totalBoids; i++) {
				int x = (int) rando.gauss(xctr, width/2);
				int y = (int) rando.gauss(yctr, height/2);
				addOneBoid(x, y);
			}
			break;
		}
		case GRID: {
			int q = 1;
			while (q * q < totalBoids) q++;
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
			for (int i = 0; i < totalBoids; i++) {
				PVector loc = gridPoints.get(i);
				addOneBoid(loc.x, loc.y);
			}
			break;
		}
		default: {
			for (int i = 0; i < totalBoids; i++) {
				addOneBoid(random(0, width), random(0, height));
			}
		}
		}
	}
	
	/**
	 * Adds one boid to the flock.
	 */
	public void addOneBoid(float x, float y) {
		// TurtleBoid tBoid = new TurtleBoid(this, new PVector(width/2, height/2), 5.0f, 0.08f);
		TurtleBoid tBoid = new TurtleBoid(this, new PVector(x, y), 2.0f, 0.05f);
		tBoid.setVisible(isShowBoids);
		tBoid.setResponder(responder);
		float m = (random(0.9f, 1.1f) + random(0.9f, 1.1f) + random(0.9f, 1.1f)) / 3.0f;
		// float m = (random(5.1f, 9.5f) + random(5.1f, 9.5f) + random(5.1f, 9.5f)) / 3.0f;
		tBoid.setMass(m);
		float separation = (float) rando.gauss(sep, 0.005);
		if (separation > 0) tBoid.setSeparationDistance(separation);
		else tBoid.setSeparationDistance(sep);
		float alignment = (float) rando.gauss(align, 0.005);
		if (alignment > 0) tBoid.setAlignmentDistance(alignment);
		else tBoid.setAlignmentDistance(align);
		float cohesion = (float) rando.gauss(coh, 0.005);
		if (cohesion > 0) tBoid.setCohesionDistance(cohesion);
		else tBoid.setCohesionDistance(coh);
		tBoid.setDisplaying(flockIsDisplaying);
		Turtle t = tBoid.getTurtle();
		// fewer trails, less memory required
		t.setMaxTrails(4);
		t.setWeight(weight);
		float w = (float) rando.gauss(weight, 0.01);
		if (w > 0) t.setWeight(w);
		t.setNoFill();
		setShapeStroke(tBoid, targetHue, 30);
		if (!flockIsDrawing) t.penUp();
		flock.addBoid(tBoid);
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
	 float[] blueForceValues = {13, 21, 29, 34, 47, 55, 76, 89, 110, 123};
	 public void initBlueNoise() {
		float scale = width/1830;
		for (int i = 0; i < bluenoise.length/2; i++) {
			float x = bluenoise[2 * i] * scale;
			float y = bluenoise[2 * i + 1] * scale;
			blueVectors.add(new PVector(x, y));
			println("blue vectors element "+ i +" = "+ x +", "+ y);
		}
	 }
	 
	 public void initMask() {
		 maskImage = this.createImage(width, height, ARGB);
		 int maskColor = color(64, 64, 64, 64);
		 maskImage.loadPixels();
		 for (int i = 0; i < width * height; i++) {
			 maskImage.pixels[i] = maskColor;
		 }
	 }
	
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
		println("Press 'h' to show this help message"); 
		// Video-tracking/Optical flow controls:
		println("Press 'f' to show or hide flow lines");
		println("Press 'i' to show or hide video image");
		println("Motion in upper right corner increases boid separation (green dot appears)");
		println("Motion in upper left corner decreases boid separation (red dot appears)");
	}

	// TODO fix flickering
	public void draw() {
		// exit loop instead of calling noLoop() so P5 and other calls can still propagate
		if (isPaused) return;
		optical.flow();
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
		if (dashedLine /* && 20 == step % 21 */) {
			// toggleDrawing();
			GroupComponent g = new GroupComponent(this);
			for (Boid tBoid : flock.getBoids()) {
				Turtle t = ((TurtleBoid) tBoid).getTurtle();
				float x = (float) t.getTurtleX();
				float y = (float) t.getTurtleY();
				// int farb = t.strokeColor();
				// BezShape blob = this.createBlob(5, 2, 0.01f, 0.01f, 0.01f, random(0, PI/4), x, y, farb);
				BezShape blob = makeBlob(x, y);
				g.add(blob);
				blob.draw(pg);
			}
			this.gList.add(g);
			// draw blobs only once at current position
			dashedLine = false;
		}
		inc = PApplet.PI/divisor;
		// step++;
		if (frameCount % 240 == 0) {
			pg.fill(255, 10);
			pg.noStroke();
			pg.rect(0, 0, width, height);
		}
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
		}
	}
	
	public void evitar() {
		avoidance = -0.5f;
		for (Boid tBoid : flock.getBoids()) {
			PVector vec = optical.getFlow(tBoid.getLoc());
			// tBoid.avoid(vec, avoidance);
			tBoid.applyForce(vec);
		}
	}

	public void mousePressed() {
		if (controlP5.isVisible()) return;
		for (Boid tBoid : flock.getBoids()) {
			tBoid.arrive(new PVector(mouseX, mouseY));
		}
	}
	
	public void mouseDragged() {
		if (controlP5.isVisible()) return;
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
	
	// key presses TODO
	public void keyPressed() {
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
		else if (key == 'k' || key == 'K') {
			stopDrawing();
		}
		else if (key == 'x' || key == 'X') {
			erase();
			gList.clear(); 
		}
		else if (key == 'p' || key == 'P') {
			isPaused = !isPaused;
		}
		else if (key == 's' || key == 'S') {
			stopDrawing();
			saveListAI("turtleboids");
		}
		else if (key == 'y' || key == 'Y') {
			// reset the divisor
			divisor = 128;
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
		else if (key == CODED) {
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
		else if (key == 'n' || key == 'N') {
			newBoids();
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
			optical.toggleFlowDisplay();
		}
		else if (key == 'i' || key == 'I') {
			isShowVideo = !isShowVideo;
			if (isShowVideo) {
				optical.showImage();
			}
			else {
				optical.hideImage();
			}
		}
		else if (key == '1') {
			skipAction = !skipAction;
			println("skipAction = "+ skipAction);
		}
		else if (key == 'a' || key == 'A') {
			selectedBoidState = (selectedBoidState + 1) % boidStateList.size();
			assignBoidState(selectedBoidState, 1f);
			println("selectedBoidState: "+ selectedBoidState);
		}
		else if (key == 'q' || key == 'Q') {
			placement = BoidPlacement.values()[rando.randomInRange(0, BoidPlacement.values().length - 1)];
			println("placement = "+ placement.toString());		}
	}
	
	public void startDrawing() {
		// targetHue = (targetHue + 5) % 360;
		for (Boid tBoid : flock.getBoids()) {
			((TurtleBoid) tBoid).getTurtle().penDown();
			// maintain a more constant hue
			// setShapeStroke(((TurtleBoid) tBoid), targetHue, 30);
		}
	}
	
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
		bgRect.setFillColor(0);
		LayerComponent bgLayer = new LayerComponent(this, "background");
		bgLayer.add(bgRect);
		bgLayer.setLocked(true);
		// begin adding components to boidsLayer
		LayerComponent boidsLayer = new LayerComponent(this, "boids");
/* 		// the layering used here puts trails from each boid into its own group, not the same as the display
		// the getTrails() method returns a turtle's trails, saved and current, bundled into a GroupComponent
		// the GroupComponent can be added to a DocumentComponent, LayerComponent, or another GroupComponent
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
		filename = filename +"_"+ fileCount++ +".ai";
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
		bgRect.setFillColor(0);
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
		int yPos = 4;
		int step = 18;
		int widgetH = 14;
		int labelW = 144;
		settings = controlP5.addGroup("Global Settings", 4, 20, settingsWidth);
		settings.setBackgroundColor(panelBack);
		settings.setBackgroundHeight(240);
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
		pg.background(255);
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
	/**
	 * @param colorShape
	 * @param targHue
	 * @param variance
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
//			if (tBoid instanceof TurtleBoid) println("it's a turtle boid");
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
//			println("call back from TurtleBoid id "+ ((TurtleBoid)tBoid).id +" with hue "+ targHue);
		}
		
		public void stepColor(Boid tBoid, Turtle t, float stepSize) {
			int tbStrokeColor = t.strokeColor();
			colorMode(HSB, 360, 100, 100);
			float targHue = hue(tbStrokeColor) + 15;
			colorMode(RGB, 255, 255, 255);
			setShapeStroke(((TurtleBoid)tBoid), targHue, 30);
		}
	}
	
	
	boolean skipAction = true;
	class VideoResponder implements VideoCallbackINF {
		float actionThreshold = 0.5f;
		float sepMax = 233;
		float sepMin = 3.0f;
		int evtTimer;
		int evtDebounce = 120;
		PVector btn1 = new PVector(50, 30);
		PVector btn2 = new PVector(width/3, 30);
		PVector btn3 = new PVector(2 * width/3, 30);
		PVector btn4 = new PVector(width - 30, 30);
		
		@Override
		public void videoCallback(Capture video) {
			// TODO fix flickering -- 
			/*
			// this works but requires video to be showing 
			background(pg);
			video.mask(maskImage);
			blend(video, 0, 0, width, height, 0, 0, width, height, BLEND);
			 */
			// pg.set(0, 0, video);
			background(pg);
			video.mask(maskImage);
			blend(video, 0, 0, width, height, 0, 0, width, height, BLEND);
			// use if display is resized
			/*
			image(pg, 0, 0, displayWidth, displayHeight);
			video.mask(maskImage);
			blend(video, 0, 0, width, height, 0, 0, displayWidth, displayHeight, BLEND);
			 */
		}
		
		@Override
		public void vectorCallback(Capture video) {
			// method stub. OpticalFlowMaker instance will draw the vector field for us.
		}

		@Override
		public void actionCallback(Capture video) {
			if (skipAction) return;
			// trigger call to setSeparation() by setting the number box, avoid recursion
			Numberbox n1 = (Numberbox) controlP5.getController("setSeparation");
			float mag1 = optical.getFlow(btn1).mag();
			float mag2 = optical.getFlow(btn2).mag();
			float mag3 = optical.getFlow(btn3).mag();
			float mag4 = optical.getFlow(btn4).mag();
			int color1 = color(233, 89, 55);
			int color2 = color(55, 233, 144);
			int color3 = color(55, 21, 233);
			int color4 = color(233, 199, 55);
			int color5 = color(216, 216, 216);
			if (mag1 > actionThreshold) {
				optical.markGrid((int)btn1.x, (int)btn1.y, color5);
				if (mag2 > actionThreshold) {
					optical.markGrid((int)btn2.x, (int)btn2.y, color5);
					if (millis() - evtTimer < evtDebounce) return;
					n1.setValue(max(sep - 1, sepMin));
					evtTimer = millis();
				}
				else if (mag4 > actionThreshold) {
					optical.markGrid((int)btn4.x, (int)btn4.y, color5);
					if (millis() - evtTimer < evtDebounce) return;
					assignBoidState(rando.randomInRange(0, boidStateList.size() - 1), 1f);
					evtTimer = millis();
				}
			}
			else if (mag2 > actionThreshold) {
				optical.markGrid((int)btn2.x, (int)btn2.y, color5);
				if (mag3 > actionThreshold) {
					optical.markGrid((int)btn3.x, (int)btn3.y, color5);
					if (millis() - evtTimer < evtDebounce) return;
					erase();
					evtTimer = millis();
				}
			}
			else if (mag3 > actionThreshold) {
				optical.markGrid((int)btn3.x, (int)btn3.y, color5);
				if (mag4 > actionThreshold) {
					optical.markGrid((int)btn4.x, (int)btn4.y, color5);
					if (millis() - evtTimer < evtDebounce) return;
					n1.setValue(min(sep + 1, sepMax));
					evtTimer = millis();
				}
			}
			adjustFlock();
		}
	}
	
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
	

