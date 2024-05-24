### unit6_exercises

#### Basic subscriber, no template

[`subscriber_basic.h`](src/subscriber_basic.h)  
[`subscriber_basic.cpp`](src/subscriber_basic.cpp)  
[`main_subscriber_basic.cpp`](src/main_subscriber_basic.cpp)  

#### Function templates

Topic: `/camera/rgb/image_raw`  
Data type: [`sensor_msgs/Image`](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)  

[`topic_subscriber_function.h`](src/topic_subscriber_function.h)  
[`topic_subscriber_function.cpp`](src/topic_subscriber_function.cpp)  
[`topic_subscriber_function_node_main.cpp`](src/topic_subscriber_function_node_main.cpp)  

Topic: `/camera/depth_registered/points`  
Data type: [`sensor_msgs/PointCloud2`](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html)  

[`topic_subscriber_function.h`](src/topic_subscriber_function.h)  
[`topic_subscriber_function.cpp`](src/topic_subscriber_function.cpp)  
[`topic_subscriber_function_node_main.cpp`](src/topic_subscriber_function_node_main.cpp)  


#### Class template

[`topic_subscriber_class.h`](src/topic_subscriber_class.h)  
[`topic_subscriber_class.cpp`](src/topic_subscriber_class.cpp)  
[`topic_subscriber_class_node_main.cpp`](src/topic_subscriber_class_node_main.cpp)  

#### Callback specialization specialised_example

[`specialized_example.h`](src/specialized_example.h)  
[`specialized_example.cpp`](src/specialized_example.cpp)  
[`main_specialized_example.h`](src/main_specialized_example.cpp)  

#### Edge detector callback specialization

[`magic_subscriber_image_edgedetector.h`](src/magic_subscriber_image_edgedetector.h)  
[`magic_subscriber_image_edgedetector.cpp`](src/magic_subscriber_image_edgedetector.cpp)  
[`main_edge detector.cpp`](src/main_edge detector.cpp)  

#### Depth from point cloud specialization

[`magic_subscriber_pcl2image.h`](src/magic_subscriber_pcl2image.h)  
[`magic_subscriber_pcl2image.cpp`](src/magic_subscriber_pcl2image.cpp)  
[`main_pcl2image.h`](src/main_pcl2image.h)  

