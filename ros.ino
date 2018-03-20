
ros::NodeHandle nh;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

char probe_tip[] = "/probe_tip";
char probe_base[] = "/probe_base";

probe::probe_data probe_status_reply;
ros::Publisher probe_status_reply_pub( "probe_status_reply", &probe_status_reply);
probe::probe_data probe_contact_reply;
ros::Publisher probe_contact_reply_pub( "probe_contact_reply", &probe_contact_reply);

void probeCmdClbk(const std_msgs::Int16& msg);
ros::Subscriber<std_msgs::Int16> probe_cmd_sub("probe_cmd_send", probeCmdClbk);

void setupROS() {
  nh.initNode();
  broadcaster.init(nh);
  nh.subscribe(probe_cmd_sub);
  setupMsg(probe_status_reply, probe_status_reply_pub);
  setupMsg(probe_contact_reply, probe_contact_reply_pub);
}

// Data Rate Attenuation
long int lastPubTime = 0;
int dataRate  = 20; // ms or 50Hz

void runROS() {
  long int time = millis();
  double dt = (time - lastPubTime);
  if (dt > dataRate) {

    // probe msg
    sendMsg(probe_status_reply, probe_status_reply_pub);
    lastPubTime = time;

    // tf probe_base->probe_tip
    t.header.frame_id = probe_base;
    t.child_frame_id = probe_tip;
    t.transform.translation.x = getMotorPosition();
    t.transform.rotation = tf::createQuaternionFromYaw(0);
    t.header.stamp = nh.now();
    broadcaster.sendTransform(t);
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








