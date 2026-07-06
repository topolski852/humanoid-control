"""
Optional shared-password access control (adapted from humanoid-hand).

If ``HUMANOID_WEB_PASSWORD`` is set, every mutating API route and websocket requires a
bearer token obtained by POSTing the password to ``/auth/login``. If it is unset (the
default — trusted-LAN use), auth is disabled entirely.

Tokens are opaque, random, and held in memory (clients re-login after a restart).
Includes a per-IP brute-force throttle.
"""
from __future__ import annotations

import hmac
import logging
import os
import secrets
import time

from fastapi import Header, HTTPException

_log = logging.getLogger(__name__)

_PASSWORD = os.environ.get("HUMANOID_WEB_PASSWORD") or None
_tokens: set[str] = set()

# ── brute-force throttle (per client IP) ─────────────────────────────────────
_MAX_FAILS = 5        # failures within the window before a lockout
_WINDOW = 60.0        # seconds
_LOCKOUT = 300.0      # seconds locked out after too many failures
_fails: dict[str, list[float]] = {}
_locked_until: dict[str, float] = {}

if _PASSWORD is not None and len(_PASSWORD) < 12:
    _log.warning("HUMANOID_WEB_PASSWORD is short (<12 chars) — use a strong password before "
                 "exposing this beyond a trusted LAN.")


def auth_required() -> bool:
    return _PASSWORD is not None


def login_locked(ip: str) -> bool:
    return time.time() < _locked_until.get(ip, 0.0)


def record_login_failure(ip: str) -> None:
    now = time.time()
    arr = [t for t in _fails.get(ip, []) if now - t < _WINDOW]
    arr.append(now)
    _fails[ip] = arr
    if len(arr) >= _MAX_FAILS:
        _locked_until[ip] = now + _LOCKOUT
        _fails[ip] = []
        _log.warning("Login locked out for %s after %d failed attempts", ip, _MAX_FAILS)


def record_login_success(ip: str) -> None:
    _fails.pop(ip, None)
    _locked_until.pop(ip, None)


def issue_token(password: str) -> str | None:
    """Return a fresh token if the password matches, else None."""
    if not auth_required():
        return None
    if not hmac.compare_digest(password, _PASSWORD):
        return None
    token = secrets.token_urlsafe(32)
    _tokens.add(token)
    return token


def token_valid(token: str | None) -> bool:
    return bool(token) and token in _tokens


def require_auth(authorization: str | None = Header(default=None)) -> None:
    """FastAPI dependency: 401 unless a valid bearer token is presented."""
    if not auth_required():
        return
    token = None
    if authorization and authorization.lower().startswith("bearer "):
        token = authorization[7:]
    if not token_valid(token):
        raise HTTPException(status_code=401, detail="unauthorized")
