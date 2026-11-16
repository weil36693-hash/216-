from contextlib import redirect_stdout, redirect_stderr
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import runtime_config as client


class RuntimeClientTests(unittest.TestCase):
    def status(self, device="camera", name="Maturo_CAM_AABBCC112233"):
        return {"name" if device == "camera" else "device_name": name,
                "firmware": {"version": client.RELEASE, "project": client.PROJECTS[device]}}

    def test_wrong_identity_type_or_version_rejected(self):
        for status in (self.status(name="wrong"), self.status("chassis"),
                       {**self.status(), "firmware": {"version": "old"}}):
            with self.assertRaises(ValueError):
                client.verify_identity(status, "camera", "Maturo_CAM_AABBCC112233")

    def test_chassis_project_name_alias(self):
        status = {"device_name": "Maturo_AABBCC112233", "firmware": {
            "version": client.RELEASE, "project_name": client.PROJECTS["chassis"]}}
        client.verify_identity(status, "chassis", "Maturo_AABBCC112233")

    def test_write_checks_identity_before_post(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "settings.json"
            source.write_text(json.dumps({"fps": 7}))
            with patch.object(client, "request", side_effect=[self.status(), {"saved": True}]) as request, \
                    redirect_stdout(io.StringIO()):
                result = client.main(["--device", "camera", "--host", "10.1.1.2", "set",
                                      "--expect-name", "Maturo_CAM_AABBCC112233", "--json-file", str(source)])
            self.assertEqual(result, 0)
            self.assertEqual(request.call_args_list[0].args, ("10.1.1.2", "/api/status"))
            self.assertEqual(request.call_args_list[1].args, ("10.1.1.2", "/api/settings", {"fps": 7}))

    def test_wrong_module_never_posts(self):
        with patch.object(client, "request", return_value=self.status(name="other")) as request, \
                redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            client.main(["--device", "camera", "--host", "10.1.1.2", "set",
                         "--expect-name", "Maturo_CAM_AABBCC112233", "--json-file", "does-not-exist"])
        self.assertEqual(request.call_count, 1)

    def test_output_redacts_nested_credentials(self):
        result = client.redacted({"wifi": {"password": "test-secret", "ssid": "lab"}})
        self.assertNotIn("test-secret", json.dumps(result))
        self.assertEqual(result["wifi"]["ssid"], "lab")


if __name__ == "__main__":
    unittest.main()
