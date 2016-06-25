import java.io.PrintWriter;

import com.ignofactory.steering.*;
import net.paulhertz.aifile.*;
import net.paulhertz.util.RandUtil;

import processing.core.*;


/**
 * @author Paul Hertz <ignotus@gmail.com>
 * First version of flocking application, just the basics. 
 * Based on Flocking,  by Daniel Shiffman <http://www.shiffman.net>, in The Nature of Code, Spring 2009.
 * Demonstration of Craig Reynolds' "Flocking" behavior, See: http://www.red3d.com/cwr/.
 * Rules: Cohesion, Separation, Alignment
 * 
 * Offscreen drawing is rudimentary in this early version.
 *
 */
public class Flocking extends PApplet {
	// Click mouse to add boids into the system (disabled)

	Flock flock;
	boolean flockIsDrawing;
	RandUtil rando;
	int targetHue = 60;
	BoidCallbackINF responder;
	PGraphics pg;

	/**
	 * @param args
	 * necessary method for running in Eclipse IDE
	 */
	public static void main(String[] args) {
		PApplet.main(new String[] { "--present", "Flocking" });
	}

	public void setup() {
		size(890, 550);
		smooth();
		rando = new RandUtil();
		pg = createGraphics(width, height);
		pg.beginDraw();
		pg.background(255);
		pg.endDraw();
		responder = new Responder();
		flock = new Flock();
		// Add an initial set of boids into the system
		noFill();
		flockIsDrawing = true;
		for (int i = 0; i < 144; i++) {
			// TurtleBoid tBoid = new TurtleBoid(this, new PVector(width/2, height/2), 3.0f, 0.05f);
			TurtleBoid tBoid = new TurtleBoid(this, new PVector(random(0, width), random(0, height)), 3.0f, 0.05f);
			tBoid.setResponder(responder);
			float m = random(0.8f, 1.2f);
			tBoid.setMass(m);
			float separation = 2;
			float sep = (float) rando.gauss(separation, 0.01);
			if (sep > 0) separation = sep;
			tBoid.setSeparationDistance(separation);
			tBoid.setAlignmentDistance(4 * separation);
			tBoid.setCohesionDistance(10 * separation);
			Turtle t = tBoid.getTurtle();
			t.setMaxTrails(1);
			t.setWeight(0.75f);
			float w = (float) rando.gauss(0.75, 0.01);
			if (w > 0) t.setWeight(w);
			setShapeStroke(tBoid, targetHue, 30);
			if (!flockIsDrawing) t.penUp();
			flock.addBoid(tBoid);
		}
	}

	public void draw() {
		background(255);
		image(pg, 0, 0);
		flock.run();
	}

/*	// Add a new boid into the System
	public void mousePressed() {
		TurtleBoid tBoid = new TurtleBoid(this, new PVector(mouseX, mouseY), 3.0f, 0.05f);
		tBoid.setResponder(responder);
		Turtle t = tBoid.getTurtle();
		t.setNoFill();
		setShapeStroke(tBoid, targetHue, 30);
		if (!flockIsDrawing) t.penUp();
		flock.addBoid(tBoid);
	}
*/
	
	public void mouseDragged() {
		// we use mouseDragged event to apply force to the boids, to drag them with the mouse
		PVector mLoc = new PVector(mouseX, mouseY);
		PVector wind = new PVector(mouseX - pmouseX, mouseY - pmouseY);
		wind.mult(0.1f);
		for (Boid tBoid : flock.getBoids()) {
			float dist = PVector.dist(mLoc, tBoid.getLoc());
			float k = 1.0f / (dist * dist);
			wind.mult(0.99f + k);
			tBoid.applyForce(wind);
		}
	}
	
	public void keyPressed() {
		if (key == 'd' || key == 'D') {
			flockIsDrawing = !flockIsDrawing;
			if (flockIsDrawing) {
				targetHue = (targetHue + 5) % 360;
				for (Boid tBoid : flock.getBoids()) {
					((TurtleBoid) tBoid).getTurtle().penDown();
					setShapeStroke(((TurtleBoid) tBoid), targetHue, 30);
				}
			}
			else {
				for (Boid tBoid : flock.getBoids()) {
					((TurtleBoid) tBoid).getTurtle().penUp();
				}
				drawOffscreen(pg);
			}
			println("trail drawing is "+ flockIsDrawing);
		}
		else if (key == 'r' || key == 'R') {
			TurtleBoid tBoid = null;
			for (Boid boid : flock.getBoids()) {
				tBoid = ((TurtleBoid) boid);
				tBoid.toggleDisplaying();
			}
			if (null != tBoid) println("trail displaying is "+ (tBoid).isDisplaying());
		}
		else if (key == 'c' || key == 'C') {
			targetHue = (targetHue + 5) % 360;
			for (Boid tBoid : flock.getBoids()) {
				setShapeStroke(((TurtleBoid) tBoid), targetHue, 30);
			}
		}
		else if (key == 'x' || key == 'X') {
			for (Boid tBoid : flock.getBoids()) {
				((TurtleBoid) tBoid).getTurtle().clear();
			}
		}
		else if (key == 's' || key == 'S') {
			saveAI("turtleboids.ai");
		}
	}
	
	public void drawOffscreen(PGraphics pg) {
		  println("offscreen starting...");
		  pg.beginDraw();
		  pg.background(255);
		  for (Boid tBoid : flock.getBoids()) {
		    Turtle t = ((TurtleBoid) tBoid).getTurtle();
		    t.draw(pg);
		  }
		  pg.endDraw();
		  println("offscreen done");
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
	 * @param filename   name of file to save to
	 * Saves boids trails to an Adobe Illustrator 7.0 file format using my IgnoCodeLib software.
	 */
	public void saveAI(String filename) {
		PrintWriter pw = createWriter(filename);
		DocumentComponent document = new DocumentComponent(this, "TurtleBoid Trails");
		document.setVerbose(true);
		document.setCreator("Ignotus");
		document.setOrg("IgnoStudio");
		document.setWidth(width);
		document.setHeight(height);
		BezShape bgRect = BezRectangle.makeLeftTopWidthHeight(this, 0, 0, width, height);
		bgRect.setNoStroke();
		bgRect.setFillColor(0);
		LayerComponent bgLayer = new LayerComponent(this, "background");
		bgLayer.add(bgRect);
		LayerComponent boidsLayer = new LayerComponent(this, "boids");
		// the getTrails() method returns a turtle's trails, saved and current, bundled into a GroupComponent
		// the GroupComponent can be added to a DocumentComponent, LayerComponent, or another GroupComponent
		for (Boid tBoid : flock.getBoids()) {
			Turtle t = ((TurtleBoid) tBoid).getTurtle();
			boidsLayer.add(t.getTrails());
		}
		document.add(bgLayer);
		document.add(boidsLayer);
		document.writeWithAITransform(pw);
	}

	class Responder implements BoidCallbackINF {
		public void callback(Boid tBoid) {
//			if (tBoid instanceof TurtleBoid) println("it's a turtle boid");
			Turtle t = ((TurtleBoid)tBoid).getTurtle();
			int tbStrokeColor = t.strokeColor();
			colorMode(HSB, 360, 100, 100);
			float targHue = hue(tbStrokeColor) + 15;
			colorMode(RGB, 255, 255, 255);
			setShapeStroke(((TurtleBoid)tBoid), targHue, 30);
			// add the latest trail to the offscreen graphics pg
			if (!t.isEmpty()) {
				BezShape bez = t.getTurtleTrails().get(t.size() - 1);
				pg.beginDraw();
				bez.draw(pg);
				pg.endDraw();
			}
//			println("call back from TurtleBoid id "+ ((TurtleBoid)tBoid).id +" with hue "+ targHue);
		}
	}
	
}
