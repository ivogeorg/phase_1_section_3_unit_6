#include <ros/ros.h>

template <class MyClassTemplateArg> class MyClass {
public:
  void PrintInputType(std::string in_arg) {
    std::cout << "Class instantiation type: " << typeid(in_arg).name()
              << std::endl;
  }
};

template <> class MyClass<std::string> {
public:
  void PrintInputString(std::string in_arg) {
    std::cout << "Message = " << in_arg << std::endl;
  }
};

template <typename DummyArgTemp> void Dummy_function(DummyArgTemp in_arg) {
  std::cout << "Generic dummy, specialized for " << typeid(in_arg).name() << ": " << in_arg << '\n';
}

template <> void Dummy_function<float>(float in_arg) { 
    printf("Dummy for floats: %g\n", in_arg); 
}

// NOTE: Specialization inferred from parameter type
template <> void Dummy_function(std::string in_arg) {
  std::cout << "Dummy for strings: " << in_arg << '\n';
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "basic_example_u6_specialization_node");
  ros::NodeHandle _n("basic_example_u6_specialization_ns");

  MyClass<std::string> object_1;
  object_1.PrintInputString("Adventure Time!");

  MyClass<int> object_2;
  object_2.PrintInputType("Adventure Time!");

  int dummy_value = 0;
  Dummy_function<int>(dummy_value);

  float dummy_value_f = 3.14159265359;
  Dummy_function<float>(dummy_value_f);

  std::string dummy_value_s = "Adventure Time!\n";
  Dummy_function(dummy_value_s);

  return 0;
}