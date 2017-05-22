package A;

public interface DependsOnB extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "A/DependsOnB";
  static final java.lang.String _DEFINITION = "std_msgs/String stringProperty\nB/Standalone dependentProperty ";
  std_msgs.String getStringProperty();
  void setStringProperty(std_msgs.String value);
  B.Standalone getDependentProperty();
  void setDependentProperty(B.Standalone value);
}
