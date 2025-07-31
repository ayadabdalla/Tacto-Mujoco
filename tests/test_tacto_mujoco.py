from tacto_mujoco import dummy_test
import logging
import unittest

class TestExample(unittest.TestCase):
    def test_dummy_test(self):
        result = dummy_test()
        self.assertEqual(result, "Hi, I am tacto_mujoco.__init__.py")
print("Test file is loaded!")