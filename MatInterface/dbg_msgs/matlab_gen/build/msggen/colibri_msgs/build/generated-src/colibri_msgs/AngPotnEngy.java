package colibri_msgs;

public interface AngPotnEngy extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "colibri_msgs/AngPotnEngy";
  static final java.lang.String _DEFINITION = "Header header\nfloat32 angle_min\nfloat32 angle_max\nfloat32 angle_increment\nfloat32 max_potn_engy \t\nuint16  max_index\nfloat32 pos_order_max\nuint16  pos_order_index\nfloat32 neg_order_max\nuint16  neg_order_index\nfloat32[181] potential_value\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  float getAngleMin();
  void setAngleMin(float value);
  float getAngleMax();
  void setAngleMax(float value);
  float getAngleIncrement();
  void setAngleIncrement(float value);
  float getMaxPotnEngy();
  void setMaxPotnEngy(float value);
  short getMaxIndex();
  void setMaxIndex(short value);
  float getPosOrderMax();
  void setPosOrderMax(float value);
  short getPosOrderIndex();
  void setPosOrderIndex(short value);
  float getNegOrderMax();
  void setNegOrderMax(float value);
  short getNegOrderIndex();
  void setNegOrderIndex(short value);
  float[] getPotentialValue();
  void setPotentialValue(float[] value);
}
