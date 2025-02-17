#include <math.h>

float x = 100.0;  // Test X position
float y = 100.0;  // Test Y position
float z = 50.0;   // Test Z position

struct answer {
  float theta0 = 0;
  float theta1 = 0;
  float theta2 = 0;
};

struct link_robot_t {
  float l1 = 220.0;
  float l2 = 220.0;
};

link_robot_t robot_link_1 = {};

void setup() {
  Serial.begin(115200);
  Serial.println("Test in 3 seconds");
  delay(3000);

  answer output_answer = calculate_IK(x, y, z, robot_link_1);
  
  // Print the results
  Serial.println("Inverse Kinematics Results:");
  Serial.print("Theta 0: "); Serial.println(output_answer.theta0);
  Serial.print("Theta 1: "); Serial.println(output_answer.theta1);
  Serial.print("Theta 2: "); Serial.println(output_answer.theta2);

  // Verify using Forward Kinematics
  float fk_x, fk_y, fk_z;
  calculate_FK(output_answer.theta0, output_answer.theta1, output_answer.theta2, robot_link_1, fk_x, fk_y, fk_z);

  Serial.println("\nForward Kinematics Check:");
  Serial.print("Calculated X: "); Serial.println(fk_x);
  Serial.print("Calculated Y: "); Serial.println(fk_y);
  Serial.print("Calculated Z: "); Serial.println(fk_z);

  // Compare with original values
  Serial.println("\nError Check:");
  Serial.print("Error in X: "); Serial.println(fabs(fk_x - x));
  Serial.print("Error in Y: "); Serial.println(fabs(fk_y - y));
  Serial.print("Error in Z: "); Serial.println(fabs(fk_z - z));
}

void loop() {}

answer calculate_IK(float x, float y, float z, link_robot_t robot_link_1){
  float l1 = robot_link_1.l1;
  float l2 = robot_link_1.l2;

  float w = sqrt(pow(x, 2) + pow(y, 2));
  float a = pow(z,2) + pow(w,2) - pow(l1,2) - pow(l2,2);
  float b = 2 * l1 * l2;
  float c = a / b;

  // Prevent invalid acos input
  c = constrain(c, -1.0, 1.0);

  float theta2 = acos(c);
  float a1 = l1 + (l2 * cos(theta2));
  float b1 = l2 * sin(theta2);
  float theta1 = atan2((z * a1 - w * b1), (w * a1 + z * b1));
  float theta0 = atan2(x, y);

  // Convert to degrees
  theta0 = theta0 * 180.0 / M_PI;
  theta1 = theta1 * 180.0 / M_PI;
  theta2 = theta2 * 180.0 / M_PI;

  answer output_answer = {};
  output_answer.theta0 = theta0;
  output_answer.theta1 = theta1;
  output_answer.theta2 = theta2;
  return output_answer;
}

void calculate_FK(float theta0, float theta1, float theta2, link_robot_t robot_link_1, float &out_x, float &out_y, float &out_z) {
  float l1 = robot_link_1.l1;
  float l2 = robot_link_1.l2;

  // Convert degrees back to radians
  theta0 = theta0 * M_PI / 180.0;
  theta1 = theta1 * M_PI / 180.0;
  theta2 = theta2 * M_PI / 180.0;

  // Forward Kinematics Equations
  float w = (l1 * cos(theta1)) + (l2 * cos(theta1 + theta2));
  out_x = w * sin(theta0);
  out_y = w * cos(theta0);
  out_z = (l1 * sin(theta1)) + (l2 * sin(theta1 + theta2));

}
