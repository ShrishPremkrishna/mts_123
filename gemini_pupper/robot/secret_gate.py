"""Optional Lab 3's local password check. This is a demonstration gate, not identity authentication.

No password is shipped in the repo, prompt, declarations, replies or logs.
The robot owner provisions a UTF-8 file separately. Unlocking is volatile:
restart, lie_down, stop, fall and emergency stop lock it again.
"""

from __future__ import annotations

import hmac
import os
from pathlib import Path


class SecretGate:
    def __init__(self, path: Path | None = None):
        self.path = path or Path(os.environ.get(
            "PUPPER_SECRET_FILE", str(Path.home() / ".config/gemini-pupper/secret_password"))).expanduser()
        self.unlocked = False

    def lock(self) -> None:
        self.unlocked = False

    def unlock(self, candidate: str) -> bool:
        # Wrong attempts also revoke an existing unlock. Do not echo either value.
        self.lock()
        if not isinstance(candidate, str) or not candidate or len(candidate) > 256:
            return False
        try:
            with self.path.open("r", encoding="utf-8") as f:
                password = f.read(258).rstrip("\r\n")
        except (OSError, UnicodeError):
            return False
        if not password or len(password) > 256:
            return False
        self.unlocked = hmac.compare_digest(candidate.encode("utf-8"), password.encode("utf-8"))
        return self.unlocked
