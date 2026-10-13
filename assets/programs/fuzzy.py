import unittest


def f_and(v1: float, v2: float) -> float:
    return min(v1, v2)


def f_or(v1: float, v2: float) -> float:
    return max(v1, v2)


def f_not(value: float) -> float:
    return 1.0 - value


class Fuzzifier:
    def __init__(self, start: float, end: float):
        assert start < end
        self.start = start
        self.end = end

    def fuzzify(self, value: float) -> float:
        if value <= self.start:
            return 0.0
        elif value >= self.end:
            return 1.0
        else:
            return (value - self.start) / (self.end - self.start)        


def fuzzify(value: float, start: float, end: float) -> float:
    assert start < end
    if value <= start:
        return 0.0
    elif value >= end:
        return 1.0
    else:
        return (value - start) / (end - start)


def defuzzify(value: float, zero: float, one: float) -> float:
    return zero + value * (one - zero)    
    

class FuzzyTest(unittest.TestCase):
    def test_and_or_not(self):
        self.assertEqual(0.75, f_and(1.0, 0.75))
        self.assertEqual(1.0, f_or(1.0, 0.75))
        self.assertEqual(0.0, f_not(1.0))
        self.assertEqual(1.0, f_not(0.0))

    def test_fuzzify(self):
        for expected, height in [(1.0, 76), (0.75, 74), (0.5, 72), (0.25, 70),
                                 (0.0, 68), (1.0, 80), (0.0, 62)]:
            self.assertEqual(expected, fuzzify(height, 68, 76))

    def test_defuzzify(self):
            for inseam_size, fuzzy_height in [(36, 1.0), (34.5, 0.75), (33, 0.5),
                                              (31.5, 0.25), (30, 0.0)]:
                self.assertEqual(inseam_size, defuzzify(fuzzy_height, 30, 36))

    def test_defuzzify_short(self):
                for inseam_size, fuzzy_height in [(36, 0.0), (34.5, 0.25), (33, 0.5),
                                                  (31.5, 0.75), (30, 1.0)]:
                    self.assertEqual(inseam_size, defuzzify(fuzzy_height, 36, 30))


if __name__ == "__main__":
    unittest.main()
