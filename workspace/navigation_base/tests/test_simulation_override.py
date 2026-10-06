"""Diagnostic collision override must remain explicit, local and expiring."""
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import simulation_override as override


class SimulationOverrideTests(unittest.TestCase):
    def test_enabled_file_is_ineffective_outside_docker(self):
        with patch.object(override.Path,'exists',return_value=False):
            self.assertFalse(override.active())

    def test_scope_expiry_and_enable_are_required_together(self):
        with tempfile.TemporaryDirectory() as directory:
            marker=Path(directory)/'mode.json'
            with patch.object(override,'MARKER',marker), \
                 patch.object(override.Path,'exists',return_value=True), \
                 patch.object(override.os,'uname',return_value=NS(nodename='simulation-host')), \
                 patch.object(override.time,'time',return_value=100.):
                valid={'mode':'gazebo_collision_bypass','enabled':True,'hostname':'simulation-host','expires_unix_s':110.}
                marker.write_text(json.dumps(valid));self.assertTrue(override.active())
                for change in [{'enabled':False},{'hostname':'another-host'},{'expires_unix_s':100.},{'mode':'normal'}]:
                    marker.write_text(json.dumps({**valid,**change}));self.assertFalse(override.active())
                marker.write_text('broken');self.assertFalse(override.active())

    def test_bypass_evidence_never_claims_safety_verification(self):
        result=override.unchecked()
        self.assertFalse(result['collision_checks'])
        self.assertFalse(result['safety_verified'])
        self.assertEqual(result['scope'],'temporary_simulation_diagnostic')


if __name__=='__main__':unittest.main()
