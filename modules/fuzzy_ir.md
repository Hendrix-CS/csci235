---
layout: work
type: Module
num: 6
worktitle: Fuzzy IR Readings
---

## Fuzzy IR values

1. There are seven IR sensors aboard the iRobot Create3. Let's define the 
**span** to be the number of sensor values we wish to employ, centered on
the central sensor. If I have a span of **5**, what are the starting and 
ending indices in the list of readings? What if the span is **3**?
2. What was the most effective IR threshold you found for
obstacle identification [when we studied this previously?](modules/nodes2.html)
3. Let's define a fuzzy variable `blocked`. We can say that the robot is
fully blocked when the IR sensor reports a value at or above the threshold
you identified in the previous question. What is the **highest** IR value for
which the robot is not blocked at all? Use `curses_motor.py` to experimentally
determine a suitable value.
4. Is it preferable to consider an area `blocked` if **any** of the reported
IR values are high or if **all** of the reported IR values are high? Why?
5. Which fuzzy operator would you use to combine the fuzzified IR values into
a single `blocked` fuzzy value? Explain why, with reference to your answer to 
the previous question.
6. We're now ready to create a class to represent `blocked` variables. Each
instance needs a span and an IR sensor range. The partially completed program
below includes unit tests with actual `IrIntensityVector` objects obtained
from a robot running live. Create a new Python program 
called `fuzzify_sensors.py` and copy and paste the code below into it. Then
complete the `__init__()` and `blocked()` methods of `IrBlockingFuzzifier`, 
making sure that they pass the unit test.

```
from irobot_create_msgs.msg import IrIntensityVector, IrIntensity
import irobot_create_msgs
import std_msgs
import std_msgs.msg
import builtin_interfaces
import builtin_interfaces.msg
import fuzzy
import unittest


class IrBlockingFuzzifier:
    def __init__(self, ir_lo: int, ir_hi: int, span: int):
        # Your code here

    def blocked(self, irs: IrIntensityVector) -> float:
        # Your code here


class IrTest(unittest.TestCase):
    def test_examples(self):
        irf1 = IrFuzzifier(0, 400, 7)      
        irf2 = IrFuzzifier(0, 400, 5)      
        irf3 = IrFuzzifier(0, 400, 3)      

        example1 = irobot_create_msgs.msg.IrIntensityVector(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55541602, nanosec=449138096), frame_id='base_link'), readings=[irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55541602, nanosec=449138096), frame_id='ir_intensity_side_left'), value=1), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55541602, nanosec=449138096), frame_id='ir_intensity_left'), value=13), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55541602, nanosec=449138096), frame_id='ir_intensity_front_left'), value=41), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55541602, nanosec=449138096), frame_id='ir_intensity_front_center_left'), value=42), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55541602, nanosec=449138096), frame_id='ir_intensity_front_center_right'), value=11), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55541602, nanosec=449138096), frame_id='ir_intensity_front_right'), value=441), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55541602, nanosec=449138096), frame_id='ir_intensity_right'), value=0)])
        example2 = irobot_create_msgs.msg.IrIntensityVector(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55553057, nanosec=157440512), frame_id='base_link'), readings=[irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55553057, nanosec=157440512), frame_id='ir_intensity_side_left'), value=0), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55553057, nanosec=157440512), frame_id='ir_intensity_left'), value=3), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55553057, nanosec=157440512), frame_id='ir_intensity_front_left'), value=8), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55553057, nanosec=157440512), frame_id='ir_intensity_front_center_left'), value=10), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55553057, nanosec=157440512), frame_id='ir_intensity_front_center_right'), value=8), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55553057, nanosec=157440512), frame_id='ir_intensity_front_right'), value=217), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55553057, nanosec=157440512), frame_id='ir_intensity_right'), value=482)])
        example3 = irobot_create_msgs.msg.IrIntensityVector(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55553783, nanosec=947102639), frame_id='base_link'), readings=[irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55553783, nanosec=947102639), frame_id='ir_intensity_side_left'), value=3), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55553783, nanosec=947102639), frame_id='ir_intensity_left'), value=0), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55553783, nanosec=947102639), frame_id='ir_intensity_front_left'), value=17), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55553783, nanosec=947102639), frame_id='ir_intensity_front_center_left'), value=14), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55553783, nanosec=947102639), frame_id='ir_intensity_front_center_right'), value=12), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55553783, nanosec=947102639), frame_id='ir_intensity_front_right'), value=2), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55553783, nanosec=947102639), frame_id='ir_intensity_right'), value=0)])
        
        for (example, fs) in [
            (example1, (1.0, 1.0, 0.105)), 
            (example2, (1.0, 0.5425, 0.025)), 
            (example3, (0.0425, 0.0425, 0.0425))]:
            for irf, f in zip([irf1, irf2, irf3], fs):
                print(f"span: {len(irf.span)}; expecting {f}")
                self.assertEqual(irf.blocked(example), f)

if __name__ == '__main__':
    unittest.main()
```

## Fuzzy adjustment of linear velocity

* Create a new Python file called `fuzzy_spacer.py`. In that file, create a 
ROS2 node with the following features:
  * It subscribes to the `ir_intensity` topic.
  * Every time the callback for `ir_intensity` is invoked, it uses an
    `IrBlockingFuzzifier` object to determine the linear velocity for a 
    `TwistStamped`. The angular velocity is always zero. Use `defuzzify`
    to translate the fuzzified IR value into a linear velocity.

Answer the following questions:
* WRITE SOME DISCUSSION QUESTIONS

