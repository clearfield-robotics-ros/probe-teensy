
ros::NodeHandle nh;

probe::probe_data probe_status_reply;
ros::Publisher probe_status_reply_pub( "probe_status_reply", &probe_status_reply);
probe::probe_data probe_contact_reply;
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

void setupMsg(probe::probe_data &msg, ros::Publisher &pub) {
  nh.advertise(pub);
}

void sendMsg(probe::probe_data &msg, ros::Publisher &pub) {
  msg.state           = state;                    // Echo: probe mode
  msg.init            = calibrated;               // Flag: probe initialization status
  msg.probe_complete  = (state == IDLE ? 1 : 0);  // Flag: probe complete status
  msg.linear_position = getMotorPosition();       // Value: probe linear positon (mm)
  msg.contact_made    = objectFound();            // Flag: contact type
  pub.publish(&msg);
}

void probeCmdClbk(const std_msgs::Int16& msg) {
  setState(msg.data);
}








