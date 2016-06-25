import java.awt.geom.Point2D;
import java.io.PrintWriter;
import java.util.ArrayList;

import net.paulhertz.geom.GeomUtils;
import net.paulhertz.util.RandUtil;

import com.ignofactory.steering.*;
import net.paulhertz.aifile.*;

import controlP5.*;

import processing.core.*;


/**
 * @author paulhz
 *
 * Third version of flocking application. 
 * Based on Flocking, by Daniel Shiffman <http://www.shiffman.net>, in The Nature of Code, Spring 2009.
 * Demonstration of Craig Reynolds' "Flocking" behavior, See: http://www.red3d.com/cwr/.
 * Rules: Cohesion, Separation, Alignment
 * 
 * Requires IgnoCodeLib (see https://processing.org/reference/libraries/) and ControlP5 (http://www.sojamo.de/libraries/controlP5/).
 * 
 */
public class Flocking02 extends PApplet {
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
	PGraphics pg;
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
	float weight = 1.0f;
	int numberOfBoids = 8;
	int totalBoids = 23;
	boolean isPaused = false;
	// wind parameters
	boolean isWindy = false;
	float t = 0; 
	float divisor = 128;
	float inc = PApplet.PI/divisor;
	float windspeed = 1.0f;
	float maxWind = 3.0f;
	float minWind = 0.01f;
	float windIncrement = 0.01f;
	float turbulenceIncrement = 0.001f;
	public static final float PHI = (1 + (float) Math.sqrt(5)) * 0.5f;
	public int fileCount = 1;
	public ArrayList<GroupComponent> gList;
	public int step = 0;
	boolean stopOnEdge = false;
	boolean beginOnEdge = false;
	boolean dashedLine = false;
	boolean neonEffect = false;

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		PApplet.main(new String[] { "--present", "Flocking01" });
	}

	public void setup() {
		size(1697, 610);
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
		// use sum ridic values 2c wha hoppen
		// sep = 89; align = 34; coh = 21;  // twists
		// sep = 13; align = 8; coh = 144;  // raggedy flocks
		// sep = 55; align = 34; coh = 21;  // open tangles with paired and tripled boids
		// sep = 55; align = 34; coh = 26;  // more clusters of boids
		// sep = 89; align = 29; coh = 76;  // 
		// sep = 47; align = 29; coh = 76;  // spurts of aligned flocks that don't continue
		// sep = 21; align = 47; coh = 123; // flocks joining and separating, swirling
		// sep = 5; align = 8; coh = 610;   // skeins, if the wind don't blow steady
		// sep = 21; align = 18; coh = 240;   // draws tangled, open skeins, if the wind don't blow too hard
		// sep = 76; align = 34; coh = 89;
		// sep = 144; align = 89; coh = 377;
		totalBoids = 233;
		sep = 21; align = 110; coh = 89;
		sepFac = 55; alignFac = 76; cohFac = 144;
		initBoids();
		controlP5 = new ControlP5(this);
		loadPanel();
		controlP5.hide();
		printHelp();
		println("PHI = "+ PHI);
		gList = new ArrayList<GroupComponent>();
	}
	
	public void initBoids() {
		for (int i = 0; i < totalBoids; i++) {
			addOneBoid();
		}
	}
	
	/**
	 * Adds one boid to the flock.
	 */
	public void addOneBoid() {
		// TurtleBoid tBoid = new TurtleBoid(this, new PVector(width/2, height/2), 5.0f, 0.08f);
		TurtleBoid tBoid = new TurtleBoid(this, new PVector(random(0, width), random(0, height)), 3.0f, 0.05f);
		tBoid.setResponder(responder);
		float m = (random(0.9f, 1.1f) + random(0.9f, 1.1f) + random(0.9f, 1.1f)) / 3.0f;
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
		t.setMaxTrails(1024);
		t.setWeight(weight);
		float w = (float) rando.gauss(weight, 0.01);
		if (w > 0) t.setWeight(w);
		t.setNoFill();
		setShapeStroke(tBoid, targetHue, 30);
		if (!flockIsDrawing) t.penUp();
		flock.addBoid(tBoid);
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
	}

	public void draw() {
		// exit loop instead of calling noLoop() so P5 and other calls can still propagate
		if (isPaused) return;
		image(pg, 0, 0);
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
			for (Boid tBoid : flock.getBoids()) {
				tBoid.arrive(new PVector(mouseX, mouseY));
			}
		}
		flock.run();
		t += inc;
		if (divisor < 4096 && isWindy) divisor++;
		evitar();
		if (dashedLine && 4 == step % 5) {
			// toggleDrawing();
			GroupComponent g = new GroupComponent(this);
			for (Boid tBoid : flock.getBoids()) {
				Turtle t = ((TurtleBoid) tBoid).getTurtle();
				float x = (float) t.getTurtleX();
				float y = (float) t.getTurtleY();
				int farb = t.strokeColor();
				BezShape blob = this.createBlob(5, 2, 0.01f, 0.01f, 0.01f, random(0, PI/4), x, y, farb);
				g.add(blob);
				blob.draw(pg);
			}
			this.gList.add(g);
		}
		inc = PApplet.PI/divisor;
		step++;
	}
	
	public void evitar() {
//		PVector v0 = new PVector(1776.02f, 1433.18f); 
//		PVector v1 = new PVector(363.839f, 1581.17f); 
//		PVector v2 = new PVector(2479.49f, 56.3434f); 
//		PVector v3 = new PVector(3694.97f, 1356.12f); 
//		PVector v4 = new PVector(636.24f, 729.949f); 
//		PVector v5 = new PVector(1091.89f, 172.738f); 
//		PVector v6 = new PVector(3076.2f, 304.473f); 
//		PVector v7 = new PVector(32.6621f, 919.624f); 
//		PVector v8 = new PVector(3828.41f, 407.89f); 
//		PVector v9 = new PVector(1212.96f, 1026.8f); 
//		PVector v10 = new PVector(2420.04f, 913.82f); 
//		PVector v11 = new PVector(3124.93f, 1027.63f); 
//		PVector v12 = new PVector(1831.25f, 569.76f);
		PVector v0 = new PVector(53.5184f, 816.184f);
		PVector v1 = new PVector(681.564f, 257.355f);
		PVector v2 = new PVector(849.521f, 561.879f);
		PVector v3 = new PVector(1457.31f, 460.985f);
		PVector v4 = new PVector(466.572f, 928.629f);
		PVector v5 = new PVector(837.832f, 894.216f);
		PVector v6 = new PVector(490.899f, 607.726f);
		PVector v7 = new PVector(1249.870f, 757.120f); 
		PVector v8 = new PVector(288.425f, 183.638f);
		PVector v9 = new PVector(182.347f, 492.310f); 
		PVector v10 = new PVector(1120.750f, 385.276f);
		PVector v11 = new PVector(1115.770f, 77.2912f);
		PVector v12 = new PVector(1502.090f, 110.536f);
		float scalexy = 1; // width/4181.0f;
		v0.mult(scalexy);
		v1.mult(scalexy);
		v2.mult(scalexy);
		v3.mult(scalexy);
		v4.mult(scalexy);
		v5.mult(scalexy);
		v6.mult(scalexy);
		v7.mult(scalexy);
		v8.mult(scalexy);
		v9.mult(scalexy);
		v10.mult(scalexy);
		v11.mult(scalexy);
		v12.mult(scalexy);
		float avoidance = 0.8f;
		scalexy = width/4181.0f;
		for (Boid tBoid : flock.getBoids()) {
			if (PVector.dist(tBoid.getLoc(), v0) < 152 * scalexy) {
				tBoid.avoid(v0, avoidance);
			}
			if (PVector.dist(tBoid.getLoc(), v1) < 199 * scalexy) {
				tBoid.avoid(v1, avoidance);
			}
			if (PVector.dist(tBoid.getLoc(), v2) < 144 * scalexy) {
				tBoid.avoid(v2, avoidance);
			}
			if (PVector.dist(tBoid.getLoc(), v3) < 233 * scalexy) {
				tBoid.avoid(v3, avoidance);
			}
			if (PVector.dist(tBoid.getLoc(), v4) < 89 * scalexy) {
				tBoid.avoid(v4, avoidance);
			}
			if (PVector.dist(tBoid.getLoc(), v5) < 110 * scalexy) {
				tBoid.avoid(v5, avoidance);
			}
			if (PVector.dist(tBoid.getLoc(), v6) < 178 * scalexy) {
				tBoid.avoid(v6, avoidance);
			}
			if (PVector.dist(tBoid.getLoc(), v7) < 377 * scalexy) {
				tBoid.avoid(v7, avoidance);
			}
			if (PVector.dist(tBoid.getLoc(), v8) < 288 * scalexy) {
				tBoid.avoid(v8, avoidance);
			}
			if (PVector.dist(tBoid.getLoc(), v9) < 322 * scalexy) {
				tBoid.avoid(v9, avoidance);
			}
			if (PVector.dist(tBoid.getLoc(), v10) < 123 * scalexy) {
				tBoid.avoid(v10, avoidance);
			}
			if (PVector.dist(tBoid.getLoc(), v11) < 76 * scalexy) {
				tBoid.avoid(v11, avoidance);
			}
			if (PVector.dist(tBoid.getLoc(), v12) < 246 * scalexy) {
				tBoid.avoid(v12, avoidance);
			}
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
		else if (key == 'x' || key == 'X') {
			erase();
			gList.clear(); 
		}
		else if (key == 'p' || key == 'P') {
			isPaused = !isPaused;
		}
		else if (key == 's' || key == 'S') {
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
			for (Boid tBoid : flock.getBoids()) {
				tBoid.setVisible(!tBoid.isVisible());
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
		else if (key == 'n' || key == 'N') {
			neonEffect = !neonEffect;
			println("neon effect = "+ neonEffect);
		}
		else if (key == 'e' || key == 'E') {
			dashedLine = !dashedLine;
		}
		else if (key == 'h' || key == 'H') {
			printHelp();
		}
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
			if (neonEffect) {
				BezShape bez = t.getTurtleTrails().get(t.size() - 1);
				BezShape bezCopy = bez.clone();
				bezCopy.setWeight(bezCopy.weight() * 0.5f);
				bezCopy.setStrokeColor(color(255));
				t.getTurtleTrails().add(bezCopy);
			}
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
		n1.setLabel("");
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
		n2.setLabel("");
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
		n3.setLabel("");
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
		n0.setLabel("");
		// label for alignment factor number box
		Textlabel l0 = controlP5.addTextlabel("alignFacLabel", "SEPARATION RATIO", 112, yPos + 4);
		l0.setWidth(labelW);
		l0.setGroup(settings);
		// alignment factor
		yPos += step - 2;
		Numberbox n4 = controlP5.addNumberbox("setAlignFac", alignFac, 8, yPos, 100, widgetH);
		n4.setGroup(settings);
		n4.setMultiplier(0.25f);
		n4.setMin(0.25f);
		n4.setMax(800.0f);
		n4.setLabel("");
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
		n5.setLabel("");
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
		n6.setLabel("");
		// label for number of boids number box
		Textlabel l6 = controlP5.addTextlabel("numberOfBoidsLabel", "NUMBER OF BOIDS", 112, yPos + 4);
		l6.setWidth(labelW);
		l6.setGroup(settings);
		yPos += step;
		Button b1 = controlP5.addButton("addBoids", 0, 8, yPos, 76, widgetH);
		b1.setGroup(settings);
		b1.setLabel("Add Boids");
		Button b2 = controlP5.addButton("subtractBoids", 0, settingsWidth/3 + 4, yPos, 76, widgetH);
		b2.setGroup(settings);
		b2.setLabel("Subtract Boids");
		Button b3 = controlP5.addButton("newBoids", 0, 2 * settingsWidth/3, yPos, 76, widgetH);
		b3.setGroup(settings);
		b3.setLabel("New Boids");
		yPos += step;
		Textlabel l7 = controlP5.addTextlabel("totalBoidsLabel", "Total boids: " + totalBoids, 8, yPos + 4);
		l7.setGroup(settings);
		yPos += step;
		Button b4 = controlP5.addButton("erase", 0, 8, yPos, 76, widgetH);
		b4.setGroup(settings);
		b4.setLabel("Erase");
		Button b5 = controlP5.addButton("pauseLoop", 0, settingsWidth/3 + 4, yPos, 76, widgetH);
		b5.setGroup(settings);
		b5.setLabel("Pause");
		Button b6 = controlP5.addButton("runLoop", 0, 2 * settingsWidth/3, yPos, 76, widgetH);
		b6.setGroup(settings);
		b6.setLabel("Run");
		yPos += step + 4;
		Numberbox n8 = controlP5.addNumberbox("setStrokeWeight", weight, 8, yPos, 100, widgetH);
		n8.setGroup(settings);
		n8.setMultiplier(0.25f);
		n8.setMin(0.25f);
		n8.setMax(48.0f);
		n8.setLabel("");
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
			addOneBoid();
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
		for (int i = 0; i < totalBoids; i++) {
			addOneBoid();
		}
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
		public void callback(Boid tBoid) {
//			if (tBoid instanceof TurtleBoid) println("it's a turtle boid");
			Turtle t = ((TurtleBoid)tBoid).getTurtle();
			// add the latest trail to the offscreen graphics pg
			if (!t.isEmpty()) {
				BezShape bez = t.getTurtleTrails().get(t.size() - 1);
				pg.beginDraw();
				bez.draw(pg);
				pg.endDraw();
			}
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
	
}
