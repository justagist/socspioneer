package sonar_conv;

import org.ros.Node;
import org.ros.MessageListener;
import org.ros.Publisher;
import org.ros.NodeConfiguration;
import org.ros.exception.RosInitException;

public class Main {
  /*
   * p2os_driver outputs sonar data in its own org.ros.message.p2os_driver.SonarArray msg format
   * But rviz requires sensor_msgs.Range format
   * So we should subscribe to /sonar, listen for org.ros.message.p2os_driver.SonarArray msgs,
   * and publish the data as sensor_msgs.Range on /sonarstd
   */
   
   /*
    * Set these according to Pioneer specifications
    */
   private static final float FOV = 0.1f; // Radians
   private static final float MIN = 0.12f; // metres
   private static final float MAX = 5.00f; // metres
   /*
    * End Pioneer-specific settings
    */
   
   private static Node node;
   private static Publisher<org.ros.message.sensor_msgs.Range> publisher;
   
   public static void main(String[] args) {
     // Use a NodeConfiguration set up with default values (should be fine for our use
     // but check the setter methods provided by NodeConfiguration for advanced features)
     NodeConfiguration config = NodeConfiguration.createDefault();
     
     // Create a node
     try {
       node = new Node("sonar_conv", config);
     
     // Create publisher which will broadcast the sensor_msgs.Range messages
     publisher = node.createPublisher("/sonarstd", org.ros.message.sensor_msgs.Range.class);
     
     // Create subscriber which will listen for org.ros.message.p2os_driver.SonarArray messages
     node.createSubscriber("/sonar", new MessageListener<org.ros.message.p2os_driver.SonarArray>() {
       // Create anonymous inner class to listen for org.ros.message.p2os_driver.SonarArray types, and act on them
       public void onNewMessage(org.ros.message.p2os_driver.SonarArray msgIn) {
         // What should we do when we receive a message on topic /sonar ?
         
         for (int i = 0; i < msgIn.ranges_count; i++) {
           org.ros.message.sensor_msgs.Range msgOut = new org.ros.message.sensor_msgs.Range();
           msgOut.header = msgIn.header; // Copy the Header for data such as timestamp
           msgOut.header.frame_id = "sonar_"+i; // Insert sonar sensor number into the header
           msgOut.radiation_type = org.ros.message.sensor_msgs.Range.ULTRASOUND;
           msgOut.field_of_view = FOV;
           msgOut.min_range = MIN;
           msgOut.max_range = MAX;
           msgOut.range = (float)msgIn.ranges[i]; // Get the actual sonar range data
           publisher.publish(msgOut);
           System.out.println("published " + i + " on /sonarstd (range " + msgOut.range + ")");
         }
       }
     }, org.ros.message.p2os_driver.SonarArray.class);

     } catch(RosInitException e) {
       e.printStackTrace();
     }
     
   }
}
