import java.util.ArrayList;
import processing.core.*;
import net.paulhertz.aifile.*;

/**
 * Created March 2012 by Paul Hertz
 * TurtleBoid composes my TurtleGraphics implementation net.paulhertz.Turtle
 * with Daniel Shiffman's implementation of Craig Reynolds famous "boids" algorithm. 
 * Basically, you get boids that can draw and save their drawing to an Adobe Illustrator file.
 * 
 */
public class TurtleBoid extends Boid implements ColorableINF {
  private Turtle turtle;
  private boolean isDisplaying;
  private BoidCallbackINF responder;
  /** counter static var for assigning component IDs */
  protected static int counter = 0;
  public int id;
  public float distance;
  public float maxDistance = -1;
  

  public TurtleBoid(PApplet parent, PVector l, float ms, float mf) {
    // call the super class constructor before anything else
    super(parent, l, ms, mf);
    // initialize the turtle at the boid's initial location 
    this.turtle = new Turtle(parent, this.loc.x, this.loc.y);
    // set the turtle bearing (turtle angle) from the velocity vector
    this.turtle.setTurtleAngle(this.vel);
    // displaying is off to begin with
    isDisplaying = false;
    this.id = TurtleBoid.counter++;
    this.distance = 0;
  }
  
  public void update() {
    super.update();
    turtle.setTurtleAngle(vel);
    turtle.move(vel.mag());
    if (maxDistance > 0) {
      this.distance += vel.mag();
      testDistance();
    }
  }
  
  public void render() {
    super.render();
    if (isDisplaying && turtle.isPenDown()) {
      turtle.drawCurrent();
    }
  }
  
  // Wraparound with drawing and callback
  // overrides Boid.borders()
  /* (non-Javadoc)
   * @see com.ignofactory.steering.Boid#borders()
   */
  void borders() {
    boolean isDrawing = turtle.isPenDown();
    if (loc.x < -r) {
      loc.x = parent.width + r;
      turtle.penUp();
      turtle.setTurtleX(loc.x);
      if (isDrawing) turtle.penDown();
      if (null != responder) responder.callback(this);
    }
    if (loc.y < -r) {
      loc.y = parent.height + r;
      turtle.penUp();
      turtle.setTurtleY(loc.y);
      if (isDrawing) turtle.penDown();
      if (null != responder) responder.callback(this);
    }
    if (loc.x > parent.width + r) {
      loc.x = -r;
      turtle.penUp();
      turtle.setTurtleX(loc.x);
      if (isDrawing) turtle.penDown();
      if (null != responder) responder.callback(this);
    }
    if (loc.y > parent.height + r) {
      loc.y = -r;
      turtle.penUp();
      turtle.setTurtleY(loc.y);
      if (isDrawing) turtle.penDown();
      if (null != responder) responder.callback(this);
    }
  }

  /**
   * @return the isDisplaying
   */
  public boolean isDisplaying() {
    return isDisplaying;
  }

  /**
   * @param isDisplaying the isDisplaying to set
   */
  public void setDisplaying(boolean isDisplaying) {
    this.isDisplaying = isDisplaying;
  }

  /**
   * sets isDisplaying to inverse boolean value
   */
  public void toggleDisplaying() {
    this.isDisplaying = !this.isDisplaying;
  }

  /**
   * @param responder the responder to set
   */
  public void setResponder(BoidCallbackINF responder) {
    this.responder = responder;
  }

  /**
   * @return the turtle
   */
  public Turtle getTurtle() {
    return turtle;
  }
  
  public float getDistance() {
    return distance;
  }

  public void setDistance(float distance) {
    this.distance = distance;
  }

  public void resetDistance() {
    this.distance = 0;
  }
  
  public float getMaxDistance() {
    return maxDistance;
  }

  public void setMaxDistance(float maxDistance) {
    this.maxDistance = maxDistance;
  }
  
  public void testDistance() {
    if (maxDistance <= 0) return;
    if (this.distance > maxDistance) {
      if (this.turtle.isPenDown()) {
        this.turtle.penUp();
      }
      // else this.turtle.penDown();
      if (null != responder) responder.callback(this);
      resetDistance();
    }
  }
  
  

  /******** Delegated methods for style ********/
  

  /**
   * @return
   * @see net.paulhertz.aifile.Turtle#fillColor()
   */
  public int fillColor() {
    return turtle.fillColor();
  }

  /**
   * @param newColor
   * @see net.paulhertz.aifile.Turtle#setFillColor(int)
   */
  public void setFillColor(int newColor) {
    turtle.setFillColor(newColor);
  }

  /**
   * 
   * @see net.paulhertz.aifile.Turtle#setNoFill()
   */
  public void setNoFill() {
    turtle.setNoFill();
  }

  /**
   * @return
   * @see net.paulhertz.aifile.Turtle#fillOpacity()
   */
  public int fillOpacity() {
    return turtle.fillOpacity();
  }

  /**
   * @param opacity
   * @see net.paulhertz.aifile.Turtle#setFillOpacity(int)
   */
  public void setFillOpacity(int opacity) {
    turtle.setFillOpacity(opacity);
  }

  /**
   * @return
   * @see net.paulhertz.aifile.Turtle#hasFill()
   */
  public boolean hasFill() {
    return turtle.hasFill();
  }

  /**
   * @return
   * @see net.paulhertz.aifile.Turtle#strokeColor()
   */
  public int strokeColor() {
    return turtle.strokeColor();
  }

  /**
   * @param newColor
   * @see net.paulhertz.aifile.Turtle#setStrokeColor(int)
   */
  public void setStrokeColor(int newColor) {
    turtle.setStrokeColor(newColor);
  }

  /**
   * 
   * @see net.paulhertz.aifile.Turtle#setNoStroke()
   */
  public void setNoStroke() {
    turtle.setNoStroke();
  }

  /**
   * @return
   * @see net.paulhertz.aifile.Turtle#strokeOpacity()
   */
  public int strokeOpacity() {
    return turtle.strokeOpacity();
  }
  
  /**
   * @param opacity
   * @see net.paulhertz.aifile.Turtle#setStrokeOpacity(int)
   */
  public void setStrokeOpacity(int opacity) {
    turtle.setStrokeOpacity(opacity);
  }

  /**
   * @return
   * @see net.paulhertz.aifile.Turtle#hasStroke()
   */
  public boolean hasStroke() {
    return turtle.hasStroke();
  }

  /**
   * @param newWeight
   * @see net.paulhertz.aifile.Turtle#setWeight(float)
   */
  public void setWeight(float newWeight) {
    turtle.setWeight(newWeight);
  }

  /**
   * @return
   * @see net.paulhertz.aifile.Turtle#weight()
   */
  public float weight() {
    return turtle.weight();
  }

  
  /******** Delegated closed or open shape methods ********/
  
  /**
   * @return
   * @see net.paulhertz.aifile.Turtle#isClosed()
   */
  public boolean isClosed() {
    return turtle.isClosed();
  }
  /**
   * @param arg0
   * @see net.paulhertz.aifile.Turtle#setIsClosed(boolean)
   */
  public void setIsClosed(boolean arg0) {
    turtle.setIsClosed(arg0);
  }

  
}

