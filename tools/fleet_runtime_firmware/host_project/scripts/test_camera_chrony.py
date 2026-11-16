import unittest
from unittest.mock import patch

import configure_camera_chrony as chrony


class ChronyConfigTests(unittest.TestCase):
    def test_preserves_sources_and_is_idempotent(self):
        before = 'pool ntp.ubuntu.com iburst\nmakestep 1 3\nrtcsync\n'
        after = chrony.render_config(before)
        self.assertIn(before.strip(), after)
        self.assertEqual(chrony.render_config(after), after)
        self.assertIn('allow 10.1.1.0/24', after)
        self.assertIn('bindaddress 10.1.1.249', after)
        self.assertNotIn('local stratum', after)

    def test_removes_duplicate_managed_bind(self):
        after = chrony.render_config('bindaddress 10.1.1.249\n')
        self.assertEqual(after.count('bindaddress 10.1.1.249'), 1)

    def test_conflicting_bind_or_port_refused(self):
        for before in ('bindaddress 192.168.1.2\n', 'port 0\n', 'port 999\n'):
            with self.subTest(before=before), self.assertRaises(ValueError):
                chrony.render_config(before)

    def test_bad_managed_block_refused(self):
        with self.assertRaises(ValueError):
            chrony.render_config(chrony.BEGIN)

    def test_wrong_host_cannot_install_or_modify_config(self):
        with patch.object(chrony.os, 'geteuid', return_value=0), \
             patch.object(chrony, 'addresses', return_value={'192.168.1.2'}), \
             patch.object(chrony.subprocess, 'run') as run, \
             patch.object(chrony.Path, 'mkdir') as mkdir:
            with self.assertRaises(ValueError):
                chrony.apply()
        run.assert_not_called()
        mkdir.assert_not_called()


if __name__ == '__main__':
    unittest.main()
