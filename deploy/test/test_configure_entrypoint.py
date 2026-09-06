from __future__ import annotations

import os
import subprocess
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory


SCRIPT = Path(__file__).resolve().parents[1] / "configure_d1_manipulation.sh"


class ConfigureEntrypointTest(unittest.TestCase):
    def test_docker_fallback_is_used_when_native_enumerator_fails(self) -> None:
        with TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            bin_dir = root / "bin"
            bin_dir.mkdir()
            capture = root / "captured-device-list.txt"

            (bin_dir / "rs-enumerate-devices").write_text(
                "#!/usr/bin/env bash\nexit 1\n", encoding="utf-8"
            )
            (bin_dir / "docker").write_text(
                """#!/usr/bin/env bash
case "$1" in
  info) exit 0 ;;
  image) [[ "$2" == inspect ]] && exit 0 ;;
  run)
    cat <<'EOF'
Device Name                   Serial Number       Firmware Version
RealSense D435I               250122077729        5.17.3.10
EOF
    exit 0
    ;;
esac
exit 2
""",
                encoding="utf-8",
            )
            (bin_dir / "python3").write_text(
                """#!/usr/bin/env bash
printf '%s' "${D1_REALSENSE_DEVICE_LIST:-}" >"${D1_TEST_CAPTURE}"
""",
                encoding="utf-8",
            )
            for executable in bin_dir.iterdir():
                executable.chmod(0o755)

            environment = os.environ.copy()
            environment["PATH"] = f"{bin_dir}:{environment['PATH']}"
            environment["D1_TEST_CAPTURE"] = str(capture)
            completed = subprocess.run(
                [str(SCRIPT), "--validate"],
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                env=environment,
            )

            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertIn("RealSense D435I", capture.read_text(encoding="utf-8"))
            self.assertIn("250122077729", capture.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
