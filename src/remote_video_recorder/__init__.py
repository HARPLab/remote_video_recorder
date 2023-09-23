from . import logger_frame

try:
    import study_runner.frames.loggers
except ImportError:
    pass
else:
    study_runner.frames.loggers.register_logger(logger_frame.REMOTE_VIDEO_RECORDER_CONFIG_NAME, 
                                                logger_frame.get_remote_video_recorder)