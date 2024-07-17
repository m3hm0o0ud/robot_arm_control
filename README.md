للبدء ببرمجة الذراع الروبوتية باستخدام ROS، سوف نتبع هذه الخطوات خطوة بخطوة:

### 1. تحضير البيئة:
- **تنصيب ROS Noetic**:
  تأكد من تثبيت ROS Noetic على نظامك. يمكنك اتباع التعليمات المتوفرة على [الصفحة الرسمية لـ ROS Noetic](http://wiki.ros.org/noetic/Installation).

### 2. إعداد workspace:
1. افتح الطرفية وأدخل الأوامر التالية لإنشاء workspace جديد:
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/
   catkin_make
   source devel/setup.bash
   ```
2. أضف هذا الأمر إلى ملف `~/.bashrc`:
   ```bash
   echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

### 3. إنشاء حزمة جديدة:
1. في مجلد `src` داخل workspace، أنشئ حزمة جديدة تسمى `my_robot_arm`:
   ```bash
   cd ~/catkin_ws/src
   catkin_create_pkg my_robot_arm std_msgs rospy roscpp
   ```

### 4. إعداد `CMakeLists.txt` و `package.xml`:
- افتح الملفين `CMakeLists.txt` و `package.xml` في مجلد الحزمة (`my_robot_arm`) وأضف التعديلات اللازمة.
- تأكد من أن `package.xml` يحتوي على جميع التبعيات المطلوبة.

### 5. برمجة العقد (nodes):
1. داخل مجلد `src` في الحزمة، أنشئ سكربت Python جديد (مثلًا `turtle_mover.py`).
2. أضف الكود الضروري للسكربت:
   ```python
   #! /usr/bin/env python3

   import rospy
   from geometry_msgs.msg import Twist

   def move_turtle():
       rospy.init_node('move_turtle', anonymous=True)
       pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
       rate = rospy.Rate(10)  # 10hz
       vel_msg = Twist()

       while not rospy.is_shutdown():
           vel_msg.linear.x = 2.0
           vel_msg.angular.z = 1.0
           pub.publish(vel_msg)
           rate.sleep()

   if __name__ == '__main__':
       try:
           move_turtle()
       except rospy.ROSInterruptException:
           pass
   ```
3. اجعل السكربت قابلًا للتنفيذ:
   ```bash
   chmod +x src/turtle_mover.py
   ```

### 6. تحديث `CMakeLists.txt`:
- أضف السطر التالي إلى `CMakeLists.txt` لتضمين السكربت:
  ```cmake
  catkin_install_python(PROGRAMS
    src/turtle_mover.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  ```

### 7. بناء workspace:
1. عد إلى مجلد `catkin_ws` وشغل الأوامر التالية:
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

### 8. تشغيل العقدة:
1. شغل ROS master:
   ```bash
   roscore
   ```
2. في نافذة طرفية أخرى، شغل عقدة `turtlesim`:
   ```bash
   rosrun turtlesim turtlesim_node
   ```
3. في نافذة طرفية أخرى، شغل العقدة التي برمجتها:
   ```bash
   rosrun my_robot_arm turtle_mover.py
   ```

الآن يجب أن ترى السلحفاة تتحرك على الشاشة.

### **التحكم بالذراع الروبوتية باستخدام MoveIt!**:
1. **تنصيب MoveIt!**:
   ```bash
   sudo apt install ros-noetic-moveit
   ```
2. **إعداد MoveIt!**:
   يمكنك إعداد ملفات التكوين لـ MoveIt! باستخدام `moveit_setup_assistant`.

3. **برمجة الحركات**:
   يمكنك برمجة الحركات الخاصة بالذراع الروبوتية باستخدام MoveIt! وPython أو C++، ويعتمد ذلك على الذراع الروبوتية التي تستخدمها.

لتفاصيل إضافية، راجع [وثائق MoveIt! الرسمية](https://moveit.ros.org/documentation/).
