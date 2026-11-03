"""Optional Lab 3's local password check. This is a demonstration gate, not identity authentication.

No password is shipped in the repo, prompt, declarations, replies or logs.
The robot owner provisions a UTF-8 file separately. Unlocking is volatile:
restart, lie_down, stop, fall and emergency stop lock it again.

The password arrives as words Gemini heard, so the comparison ignores case,
outer spaces and outer punctuation ("Bananas." matches "bananas"), but never
accepts a sentence that only contains the password.
"""

from __future__ import annotations

import hmac
import os
import unicodedata
from pathlib import Path

_EDGE = " \t\r\n.,!?;:'\"“”‘’"  # spoken-password noise at either end


def normalize(text: str) -> str:
    return unicodedata.normalize("NFKC", text).casefold().strip(_EDGE)


class SecretGate:
    def __init__(self, path: Path | None = None):
        self.path = path or Path(os.environ.get(
            "PUPPER_SECRET_FILE", str(Path.home() / ".config/gemini-pupper/secret_password"))).expanduser()
        self.unlocked = False

    def lock(self) -> None:
        self.unlocked = False

    def _password(self) -> str | None:
        """The normalized password from the file, or None if it is missing or unusable."""
        try:
            with self.path.open("r", encoding="utf-8-sig") as f:  # -sig: an editor's byte-order mark
                raw = f.read(1100).rstrip("\r\n")
        except (OSError, UnicodeError):
            return None
        if not raw or len(raw) > 256:  # an overlong file is unusable, not a shorter password
            return None
        return normalize(raw) or None

    def unlock(self, candidate: str) -> bool:
        # Wrong attempts also revoke an existing unlock. Do not echo either value.
        self.lock()
        if not isinstance(candidate, str) or not candidate or len(candidate) > 256:
            return False
        password = self._password()
        if password is None:
            return False
        self.unlocked = hmac.compare_digest(normalize(candidate).encode("utf-8"), password.encode("utf-8"))
        return self.unlocked

    def appears_in(self, text: str) -> bool:
        """True if the password occurs anywhere in text (a leak check; never returns it)."""
        password = self._password()
        return password is not None and password in unicodedata.normalize("NFKC", text).casefold()
