headings = [90 - i * 30 for i in range(7)]

def invert(nums: list[int]) -> list[int]:
    top = max(nums)
    return [top - n for n in nums]

def extract_leading_digits(s: str) -> s:
    end = 0
    while end < len(s) and s[end].isdigit():
        end += 1
    return s[:end]


def find_values(s: str) -> list[int]:
    pattern = "value="
    result = []
    start = 0
    while True:
        f = s.find(pattern, start)
        if f == -1:
            return result
        else:
            result.append(int(extract_leading_digits(s[f + len(pattern):])))
            start = f + 1 


def weighted_average_heading(irs: list[int]) -> float:
    weights = [ir * heading for (ir, heading) in zip(irs, headings)]
    return sum(weights) / sum(irs)


if __name__ == '__main__':
    example1 = "irobot_create_msgs.msg.IrIntensityVector(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55541602, nanosec=449138096), frame_id='base_link'), readings=[irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55541602, nanosec=449138096), frame_id='ir_intensity_side_left'), value=1), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55541602, nanosec=449138096), frame_id='ir_intensity_left'), value=13), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55541602, nanosec=449138096), frame_id='ir_intensity_front_left'), value=41), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55541602, nanosec=449138096), frame_id='ir_intensity_front_center_left'), value=42), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55541602, nanosec=449138096), frame_id='ir_intensity_front_center_right'), value=11), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55541602, nanosec=449138096), frame_id='ir_intensity_front_right'), value=441), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55541602, nanosec=449138096), frame_id='ir_intensity_right'), value=0)])"

    example2 = "irobot_create_msgs.msg.IrIntensityVector(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55553057, nanosec=157440512), frame_id='base_link'), readings=[irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55553057, nanosec=157440512), frame_id='ir_intensity_side_left'), value=0), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55553057, nanosec=157440512), frame_id='ir_intensity_left'), value=3), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55553057, nanosec=157440512), frame_id='ir_intensity_front_left'), value=8), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55553057, nanosec=157440512), frame_id='ir_intensity_front_center_left'), value=10), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55553057, nanosec=157440512), frame_id='ir_intensity_front_center_right'), value=8), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55553057, nanosec=157440512), frame_id='ir_intensity_front_right'), value=217), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55553057, nanosec=157440512), frame_id='ir_intensity_right'), value=482)])"

    example3 = "irobot_create_msgs.msg.IrIntensityVector(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55553783, nanosec=947102639), frame_id='base_link'), readings=[irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55553783, nanosec=947102639), frame_id='ir_intensity_side_left'), value=3), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55553783, nanosec=947102639), frame_id='ir_intensity_left'), value=0), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55553783, nanosec=947102639), frame_id='ir_intensity_front_left'), value=17), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55553783, nanosec=947102639), frame_id='ir_intensity_front_center_left'), value=14), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55553783, nanosec=947102639), frame_id='ir_intensity_front_center_right'), value=12), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55553783, nanosec=947102639), frame_id='ir_intensity_front_right'), value=2), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=55553783, nanosec=947102639), frame_id='ir_intensity_right'), value=0)])"

    for ex in [example1, example2, example3]:
        values = find_values(ex)
        print([p for p in zip(headings, values)])
        print(weighted_average_heading(values))
        inverted = invert(values)
        print(weighted_average_heading(inverted))
        print()
