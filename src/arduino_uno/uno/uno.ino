
#include <ros.h>
#include <std_msgs/Int32.h>
#include <Servo.h>

ros::NodeHandle  nh;

Servo myservo;
int n;
int distance;

// run motor with specific spee
void runset(int speed){
  int speedoutput = 0;

  // 计算速度
  if(speed>= 0 && speed<= 100){
    speedoutput = (speed/100.0)*255;
  }
  else{
    speedoutput = 0;
  }
  // 速度输出，控制变速引脚
  analogWrite(3,speedoutput);
  analogWrite(5,speedoutput);

}

// transform servo angle from rosnode to servo angle of YF-6130MG steering engine
int angle_trans(int servo) {
   return 90 - servo; 
}

// control servo angle from ros topic
void messageCb( const std_msgs::Int32 & msg){
  int steerfinal = msg.data;
  if (steerfinal > 45)
    steerfinal = 45;
  if (steerfinal < -45)
    steerfinal = -45;
  steerfinal = angle_trans(steerfinal);
  Serial.print(steerfinal);
  //  myservo.write(steerfinal);
  myservo.write(steerfinal);

}

// get speed info from ros topic
// speed information if only "forward", "backward" and "stop"
void messageSp( const std_msgs::Int32 & msg2){
  Serial.print("speed ");
  Serial.print(msg2.data);
  Serial.print("\n");
  if (msg2.data == 1){
    // 右轮
    digitalWrite(2,HIGH);
    digitalWrite(4,LOW);
    // 左轮
    digitalWrite(7,HIGH);
    digitalWrite(8,LOW);
    // unset(20);
    return;
  }
  if (msg2.data == 2){
    // 右轮
    digitalWrite(2,LOW);
    digitalWrite(4,HIGH);

    // 左轮
    digitalWrite(7,LOW);
    digitalWrite(8,HIGH);
    // runset(20);
    return;
  }
  else{
    // 右
    digitalWrite(2,LOW);
    digitalWrite(4,LOW);
    // 左
    digitalWrite(7,LOW);
    digitalWrite(8,LOW);
    return;
  }
}


ros::Subscriber<std_msgs::Int32> sub1("servo", messageCb );
ros::Subscriber<std_msgs::Int32> sub2("speed", messageSp );

// arduino setup
void setup()
{
  Serial.begin(57600);
  // attach steering engine on pin 10 for pwm output
  myservo.attach(10);
  myservo.write(90);
  // initial rosnode and subscribe sub1 and sub2
  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);

  // set pins mode
  // motors enabled pin
  pinMode(2,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  // motors pwm pin
  pinMode(3,OUTPUT);
  pinMode(5,OUTPUT);
  // steering engine pwm pin
  pinMode(10, OUTPUT);
  // 右轮
  digitalWrite(2,LOW);
  digitalWrite(4,LOW);
  // 左轮
  digitalWrite(7,LOW);
  digitalWrite(8,LOW);
}

void loop()
{
  runset(40);
  nh.spinOnce();
  // delay(50);
}


