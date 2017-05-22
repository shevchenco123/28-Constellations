package colibri_msgs;

public interface EnvSecurity extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "colibri_msgs/EnvSecurity";
  static final java.lang.String _DEFINITION = "Header header\nstd_msgs/Bool collision\nfloat32 collision_prob\nfloat32 laser_min_dis\nuint16 laser_min_index\nfloat32 laser_prob\nfloat32 ultra_min_dis\nuint16 ultra_min_index\nfloat32 ultra_prob\nfloat32 bumper_min_dis\nuint16 bumper_min_index\nfloat32 bumper_prob\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  std_msgs.Bool getCollision();
  void setCollision(std_msgs.Bool value);
  float getCollisionProb();
  void setCollisionProb(float value);
  float getLaserMinDis();
  void setLaserMinDis(float value);
  short getLaserMinIndex();
  void setLaserMinIndex(short value);
  float getLaserProb();
  void setLaserProb(float value);
  float getUltraMinDis();
  void setUltraMinDis(float value);
  short getUltraMinIndex();
  void setUltraMinIndex(short value);
  float getUltraProb();
  void setUltraProb(float value);
  float getBumperMinDis();
  void setBumperMinDis(float value);
  short getBumperMinIndex();
  void setBumperMinIndex(short value);
  float getBumperProb();
  void setBumperProb(float value);
}
