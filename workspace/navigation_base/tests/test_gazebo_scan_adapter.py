import math
import struct
import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'slam'))
from gazebo_scan_adapter import adapt_ranges, MAPPING_RANGE_M


class ScanAdapterTests(unittest.TestCase):
    def test_only_positive_inf_is_converted_and_input_is_not_mutated(self):
        values=[math.inf,-math.inf,math.nan,0.01,0.1,2.2,11.95,12.0,13.0]+[3.0]*351
        out=adapt_ranges(values,.1,12.,'vehicle/lidar')
        self.assertEqual(values[0],math.inf)
        self.assertTrue(math.isnan(out[2]))
        self.assertEqual(out[1],-math.inf)
        self.assertEqual(out[3:],values[3:])
        serialized=struct.unpack('<f',struct.pack('<f',out[0]))[0]
        self.assertGreater(serialized,MAPPING_RANGE_M)
        self.assertLess(serialized,12.)

    def test_rejects_unverified_sensor_metadata(self):
        for lo,hi,frame,values in [(.1,30.,'vehicle/lidar',[math.inf]*360),
                                  (.1,12.,'other',[math.inf]*360),
                                  (math.nan,12.,'vehicle/lidar',[math.inf]*360),
                                  (.1,12.,'vehicle/lidar',[math.inf]*720)]:
            with self.assertRaises(ValueError):adapt_ranges(values,lo,hi,frame)


if __name__=='__main__':unittest.main()
