import unittest
from training.device import resolve_device

class DeviceTests(unittest.TestCase):
    def test_cpu_even_when_cuda_exists(self):
        self.assertEqual(resolve_device('cpu', True), 'cpu')
    def test_auto(self):
        self.assertEqual(resolve_device('auto', False), 'cpu')
        self.assertEqual(resolve_device('auto', True), 'cuda')
    def test_explicit_cuda_never_silently_falls_back(self):
        with self.assertRaises(ValueError): resolve_device('cuda', False)
        self.assertEqual(resolve_device('cuda', True), 'cuda')
