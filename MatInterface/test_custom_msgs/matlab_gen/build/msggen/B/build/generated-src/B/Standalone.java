package B;

public interface Standalone extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "B/Standalone";
  static final java.lang.String _DEFINITION = "uint32 intProperty\nstring stringProperty\n";
  int getIntProperty();
  void setIntProperty(int value);
  java.lang.String getStringProperty();
  void setStringProperty(java.lang.String value);
}
