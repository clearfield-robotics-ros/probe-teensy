
ros::NodeHandle nh;

static int messageLength = 5;
std_msgs::Int16MultiArray probe_status_reply;
ros::Publisher probe_status_reply_pub( "probe_status_reply", &probe_status_reply);
std_msgs::Int16MultiArray probe_contact_reply;
ros::Publisher probe_contact_reply_pub( "probe_contact_reply", &probe_contact_reply);

void probeCmdClbk(const std_msgs::Int16& msg);
ros::Subscriber<std_msgs::Int16> probe_cmd_sub("probe_cmd_send", probeCmdClbk);

void setupROS() {
  nh.initNode();
  nh.subscribe(probe_cmd_sub);
  setupMsg(probe_status_reply, probe_status_reply_pub);
  setupMsg(probe_contact_reply, probe_contact_reply_pub);
}

// Data Rate Attenuation
long int lastPubTime = 0;
int dataRate  = 100; // ms or 10Hz

void runROS() {
  long int time = millis();
  double dt = (time - lastPubTime);
  if (dt > dataRate) {
    sendMsg(probe_status_reply, probe_status_reply_pub);
    lastPubTime = time;
  }
  nh.spinOnce();
}

void ROSContactMsg() {
  sendMsg(probe_contact_reply, probe_contact_reply_pub);
}

void setupMsg(std_msgs::Int16MultiArray &msg, ros::Publisher &pub) {
  msg.layout.dim =
    (std_msgs::MultiArrayDimension *)
    malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  msg.layout.dim[0].label = "height";
  msg.layout.dim[0].size = messageLength;
  msg.layout.dim[0].stride = 1;
  msg.layout.data_offset = 0;
  msg.data = (int16_t *)malloc(sizeof(int) * 8);
  msg.data_length = messageLength;
  nh.advertise(pub);
}

void sendMsg(std_msgs::Int16MultiArray &msg, ros::Publisher &pub) {
  msg.data[0] = state;               // Echo: probe mode
  msg.data[1] = calibrated;          // Flag: probe initialization status
  msg.data[2] = (state == IDLE ? 1 : 0); // Flag: probe complete status
  msg.data[3] = getMotorPosition();  // Value: probe linear positon (mm)
  msg.data[4] = objectFound();       // Flag: contact type
  pub.publish(&msg);
}

void probeCmdClbk(const std_msgs::Int16& msg) {
  setState(msg.data);
}








