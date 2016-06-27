import processing.video.Capture;

public interface VideoCallbackINF {
  /**
   * @param video   a Capture instance
   */
  public void videoCallback(Capture video);
  /**
   * @param video   a Capture instance
   */
  public void vectorCallback(Capture video);
  /**
   * @param video   a Capture instance
   */
  public void actionCallback(Capture video);
}



