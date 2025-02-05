#include <math.h>

struct theta {
  float a = 0.0;
  float b = 0.0;
};
struct link_robot_t {
  float l1 = 220.0;
  float l2 = 220.0;
};
struct point {
  float x, y, z;
  point(float x_ = 0, float y_ = 0, float z_ = 0) : x(x_), y(y_), z(z_) {}
};

struct answer {
  theta theta0;
  theta theta1;
  theta theta2;
};

point test_point(100.0, 100.0, 100.0);
link_robot_t robot_link_1 = {};

void setup() {
  Serial.begin(115200);
  Serial.println("Test in 3 seconds");
  delay(3000);

  answer output_answer = calculate_IK(test_point, robot_link_1);

  // Print the results
  Serial.println("Theta 0:");
  Serial.print("a: "); Serial.println(output_answer.theta0.a);
  Serial.print("b: "); Serial.println(output_answer.theta0.b);

  Serial.println("Theta 1:");
  Serial.print("a: "); Serial.println(output_answer.theta1.a);
  Serial.print("b: "); Serial.println(output_answer.theta1.b);

  Serial.println("Theta 2:");
  Serial.print("a: "); Serial.println(output_answer.theta2.a);
  Serial.print("b: "); Serial.println(output_answer.theta2.b);
}

void loop() {
  // put your main code here, to run repeatedly:
}

answer calculate_IK(point desire_point, link_robot_t robot_link_1) {
  float w = sqrt(pow(desire_point.x, 2) + pow(desire_point.y, 2));
  float alpha = atan2(w, desire_point.z);  // Use atan2 to avoid division by zero

  theta output_theta1;
  float c = pow(robot_link_1.l2, 2) - pow(w, 2) - pow(desire_point.z, 2);
  c = c / sqrt(pow(2 * robot_link_1.l1 * desire_point.z, 2) + pow(2 * robot_link_1.l1 * w, 2));

  // Ensure c is within the valid range for acos
  c = fmin(fmax(c, -1.0), 1.0);
  
  output_theta1.a = alpha + acos(c);
  output_theta1.b = alpha - acos(c);

  theta output_theta2;
  float c2a = desire_point.z - (robot_link_1.l1 * sin(output_theta1.a));
  c2a = c2a / (w - (robot_link_1.l1 * cos(output_theta1.a))); 
  output_theta2.a = atan(c2a) - output_theta1.a;

  float c2b = desire_point.z - (robot_link_1.l1 * sin(output_theta1.b));
  c2b = c2b / (w - (robot_link_1.l1 * cos(output_theta1.b))); 
  output_theta2.b = atan(c2b) - output_theta1.b;

  theta output_theta0;
  output_theta0.a = atan2(desire_point.x, desire_point.y); // Use atan2 for safety
  output_theta0.b = atan2(desire_point.x, desire_point.y);

  answer output_answer = {output_theta0, output_theta1, output_theta2};
  return output_answer;
}
