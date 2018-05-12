#include <ros.h>
#include <std_msgs/Float32.h>


ros::NodeHandle  nh;
std_msgs::Float32 gas_msg;

ros::Publisher pub_gas("gas", &gas_msg);


void setup() 
{
  nh.initNode();
  nh.advertise(pub_gas);
  Serial.begin(9600);
}

void loop() {

    float sensor_volt;
    int sensorValue = analogRead(A0);
    sensor_volt=(float)sensorValue/1024*5.0;
    Serial.println(sensor_volt);
    pub_gas.publish( &gas_msg );
    nh.spinOnce();
    //delay(1000);

}

