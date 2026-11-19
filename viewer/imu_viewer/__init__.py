__version__ = "0.1.0"

# Disable MSMF hardware transforms before OpenCV initialization: avoids long UVC startup.
import os
os.environ.setdefault("OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS", "0")
