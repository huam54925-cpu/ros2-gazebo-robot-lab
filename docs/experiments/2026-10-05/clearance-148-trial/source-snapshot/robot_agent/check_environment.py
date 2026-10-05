"""Local dependency checks and an opt-in, read-only OpenAI authentication check."""
import argparse
from importlib.metadata import version
import json
import os
from pathlib import Path
import platform

from dotenv import dotenv_values
import httpx2
import mcp
import openai

ROOT = Path(__file__).resolve().parents[2]
OFFICIAL_URL = "https://api.openai.com/v1"


def configuration(env_file=ROOT / ".env", environ=None):
    environ = os.environ if environ is None else environ
    values = dotenv_values(env_file, interpolate=False)

    def value(name, default=""):
        return (environ.get(name, values.get(name, default)) or "").strip()

    base_url = value("OPENAI_BASE_URL", OFFICIAL_URL).rstrip("/")
    if base_url != OFFICIAL_URL:
        raise ValueError("Only https://api.openai.com/v1 is supported")
    return value("OPENAI_API_KEY"), base_url, value("OPENAI_MODEL")


def check_api(key, base_url, transport=None):
    if not key:
        return {"status": "missing_api_key", "authenticated": False}, 2
    # No redirects, no retries, no generation and no robot data or commands.
    try:
        with httpx2.Client(transport=transport, follow_redirects=False) as http_client:
            with openai.OpenAI(api_key=key, base_url=base_url, timeout=20,
                               max_retries=0, http_client=http_client) as client:
                page = client.models.list()
                return {"status": "passed", "authenticated": True,
                        "models_in_first_page": len(page.data)}, 0
    except openai.APIStatusError as error:
        # Never print server messages, request headers, URLs, or key fragments.
        return {"status": "http_error", "authenticated": False,
                "http_status": error.status_code}, 3
    except openai.APITimeoutError:
        return {"status": "timeout", "authenticated": False}, 3
    except openai.APIConnectionError:
        return {"status": "connection_error", "authenticated": False}, 3
    except Exception:
        return {"status": "client_error", "authenticated": False}, 3


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--api", action="store_true",
                        help="Call official GET /v1/models; requires a local API key")
    args = parser.parse_args()
    report = {"python": platform.python_version(),
              "dependencies": {name: version(name) for name in
                               ("openai", "mcp", "python-dotenv")},
              "local_environment": "passed", "robot_interface": "read_only_installed",
              "api": {"status": "not_tested", "authenticated": False}}
    try:
        key, base_url, model = configuration()
    except ValueError:
        report["configuration"] = "invalid_official_base_url"
        print(json.dumps(report, indent=2))
        return 2
    report.update({"base_url": base_url, "api_key_configured": bool(key),
                   "model_configured": bool(model)})
    code = 0
    if args.api:
        report["api"], code = check_api(key, base_url)
    print(json.dumps(report, indent=2))
    return code


if __name__ == "__main__":
    raise SystemExit(main())
