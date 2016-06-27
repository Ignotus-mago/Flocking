// Flocking
// Daniel Shiffman <http://www.shiffman.net>
// The Nature of Code, Spring 2009
// Flock class
// Does very little, simply manages the ArrayList of all the boids

import java.util.ArrayList;

public class Flock {
    ArrayList<Boid> boids; // An arraylist for all the boids

    public Flock() {
      boids = new ArrayList<Boid>(); // Initialize the arraylist
    }

    public void run() {
      for (int i = 0; i < boids.size(); i++) {
        Boid b = (Boid) boids.get(i);  
        b.run(boids);  // Passing the entire list of boids to each boid individually
      }
    }

    public void addBoid(Boid b) {
      boids.add(b);
    }
    
    public ArrayList<Boid> getBoids() {
      return boids;
    }
}