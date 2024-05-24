#include <ros/ros.h>

// NOTE: More general/generic template first
template <typename MyClassTemplateArg> class MyClass {
public:
  void PrintInputType(std::string in_arg) {
    std::cout << "Class instantiation type: " << typeid(in_arg).name() << std::endl;
  }
};

// NOTE: Template specialization for std::string
// Specialization syntax: template <> class ClassName<SpecializationType>
template <> class MyClass<std::string> {
public:
  void PrintInputString(std::string in_arg) {
    std::cout << "Message=" << in_arg << std::endl;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "basic_example_u6_specialization_node");
  ros::NodeHandle _n("basic_example_u6_specialization_ns");

  MyClass<std::string> object_1;
  object_1.PrintInputString("Adventure Time!");

  MyClass<int> object_2;
  object_2.PrintInputType("Adventure Time!");

  return 0;
}