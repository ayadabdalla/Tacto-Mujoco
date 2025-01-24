from sim_in_sparsh import dummy_test
import logging
import unittest

class TestExample(unittest.TestCase):
    def test_dummy_test(self):
        result = dummy_test()
        self.assertEqual(result, "Hi, I am sim_in_sparsh.__init__.py")
print("Test file is loaded!")