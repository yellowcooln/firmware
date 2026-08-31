#!/usr/bin/env python3
import importlib.util
import pathlib
import unittest

SCRIPT = pathlib.Path(__file__).with_name("patch_portduino_wifi_eof.py")
spec = importlib.util.spec_from_file_location("patch_portduino_wifi_eof", SCRIPT)
assert spec is not None and spec.loader is not None
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class PatchPortduinoWifiEofTest(unittest.TestCase):
    def test_eof_keeps_connection_reset_error(self):
        source = """        if (numread == 0) // EOF
          errorCode = ECONNRESET;
        else
          errorCode = portduino_socket_errno();
        // EAGAIN (WSAEWOULDBLOCK on Windows) means timeout
        if (portduino_socket_would_block(errorCode) || numread == 0)
          errorCode = 0;
"""
        patched = module.patch_text(source)
        self.assertIn("if (portduino_socket_would_block(errorCode))", patched)
        self.assertNotIn("|| numread == 0", patched)

    def test_second_patch_is_idempotent(self):
        source = "if (portduino_socket_would_block(errorCode))\n"
        self.assertEqual(source, module.patch_text(source))


if __name__ == "__main__":
    unittest.main()
