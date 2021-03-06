#include <ros.h>
#include <std_msgs/Bool.h>

int limSwitch[3] = {16, 15, 14};
int actuatorPinHighOn = 23;
int actuatorPinHighOff = 22;
int VinPin = 6;
bool pressLim[3] = {false, false, false};

ros::NodeHandle nh;

std_msgs::Bool rosmsg[3];
ros::Publisher lim0("lim0", &rosmsg[0]);
ros::Publisher lim1("lim1", &rosmsg[1]);
ros::Publisher lim2("lim2", &rosmsg[2]);

void messageCb(const std_msgs::Bool& x) {
  if (x.data) {
    actuatorOn();
  } else {
    actuatorOff();
  }
}

ros::Subscriber<std_msgs::Bool> actuator("actuator", &messageCb );

void actuatorOn() {
    digitalWrite(actuatorPinHighOn, HIGH);
    //delay(50);
    digitalWrite(actuatorPinHighOff, LOW);
}

void actuatorOff() {
    digitalWrite(actuatorPinHighOn, LOW);
    //delay(50);
    digitalWrite(actuatorPinHighOff, HIGH);
}

void setup() {
  Serial.begin(9600);
  pinMode(VinPin, OUTPUT);
  for (int i = 0; i < 3; i++) {
    pinMode(limSwitch[i], INPUT);
  }
  pinMode(actuatorPinHighOn, OUTPUT);
  pinMode(actuatorPinHighOff, OUTPUT);
  actuatorOff();
  nh.initNode();
  nh.advertise(lim0);
  nh.advertise(lim1);
  nh.advertise(lim2);
  nh.subscribe(actuator);
}

void loop() {
  digitalWrite(VinPin, HIGH);
  for (int i = 0; i < 3; i++) {
    pressLim[i] = !digitalRead(limSwitch[i]);
    rosmsg[i].data = pressLim[i];
  }
  lim0.publish( &rosmsg[0]);
  lim1.publish( &rosmsg[1]);
  lim2.publish( &rosmsg[2]);
  nh.spinOnce();

  Serial.print("Switch_1=");
  Serial.println(pressLim[0]);
  Serial.print("Switch_2=");
  Serial.println(pressLim[1]);
  Serial.print("Switch_3=");
  Serial.println(pressLim[2]);
  delay(5);
}
