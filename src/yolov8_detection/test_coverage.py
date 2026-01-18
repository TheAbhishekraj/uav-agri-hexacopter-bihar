import unittest
import math

class TestCoverageLogic(unittest.TestCase):
    def test_spacing_calculation(self):
        # Scenario:
        # Altitude = 10m
        # FOV = 90 degrees (tan(45) = 1.0)
        # Ground Width = 2 * 10 * 1 = 20m
        # Overlap = 50% (0.5)
        # Expected Spacing = 20m * (1 - 0.5) = 10m
        
        alt = 10.0
        fov = 90.0
        overlap = 0.5
        
        fov_rad = math.radians(fov)
        ground_width = 2 * alt * math.tan(fov_rad / 2)
        spacing = ground_width * (1.0 - overlap)
        
        self.assertAlmostEqual(spacing, 10.0, places=2)
        print(f"\n✅ Test Passed: Calculated Spacing {spacing:.2f}m matches expected 10.00m")

if __name__ == '__main__':
    unittest.main()