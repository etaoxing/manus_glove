import os
import logging
import tempfile
import urllib.request
import zipfile
import shutil


logger = logging.getLogger(__name__)


_SDK_ZIP_URL = (
    "https://static.manus-meta.com/resources/manus_core_3/"
    "version_locked_installer/MANUS_Core_3.1.1_Version_Locked_Installer.zip"
)
_SDK_ZIP_MEMBER = "ManusSDK_v3.1.1/SDKMinimalClient_Linux/ManusSDK/lib/libManusSDK_Integrated.so"


def resolve_lib_path() -> str:
    """Resolve the .so path: repo-local → cache → auto-download."""
    repo_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "ManusSDK",
        "lib",
        "libManusSDK_Integrated.so",
    )
    if os.path.isfile(repo_path):
        return repo_path

    cache_path = os.path.join(
        os.path.expanduser("~"),
        ".cache",
        "manus_glove",
        "lib",
        "libManusSDK_Integrated.so",
    )
    if os.path.isfile(cache_path):
        return cache_path

    logger.info("ManusSDK .so not found locally; downloading from Manus installer…")
    download_sdk(cache_path)
    return cache_path


def download_sdk(dest: str) -> None:
    """Download the Manus installer zip and extract the .so to *dest*."""

    os.makedirs(os.path.dirname(dest), exist_ok=True)

    with tempfile.NamedTemporaryFile(suffix=".zip", delete=False) as tmp:
        tmp_path = tmp.name
    try:
        logger.info("Downloading %s …", _SDK_ZIP_URL)
        urllib.request.urlretrieve(_SDK_ZIP_URL, tmp_path)
        with zipfile.ZipFile(tmp_path) as zf:
            with zf.open(_SDK_ZIP_MEMBER) as src, open(dest, "wb") as dst:
                shutil.copyfileobj(src, dst)
        logger.info("Saved ManusSDK .so to %s", dest)
    except Exception:
        # Clean up partial file on failure
        if os.path.exists(dest):
            os.remove(dest)
        raise
    finally:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)
