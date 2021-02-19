#!/usr/bin/env python

import collections
import logging
import os
import rospy

try:
    import Tkinter as tk
except ImportError:
    import tkinter as tk # python3

try:
    from remote_video_recorder.srv import RemoteRecord, RemoteRecordRequest
    AVAILABLE = True
except ImportError:
    AVAILABLE = False

logger = logging.getLogger('remote_video_recorder')

REMOTE_VIDEO_RECORDER_CONFIG_NAME = 'remote_video_recorder'

class RemoteRecorder:
    def __init__(self, control_topic, **kwargs):
        if not AVAILABLE:
            raise RuntimeError('Remote video recorder message files not found.')

        try:
            rospy.wait_for_service(control_topic, timeout=1.)
        except Exception as ex:
            logger.warn('Failed to connect to remote video recorder: {}'.format(ex))
            self.service = None
        else:
            self.service = rospy.ServiceProxy(control_topic, RemoteRecord)
            self._req = RemoteRecordRequest(**kwargs)
         
    def start(self):
        if self.service is not None:
            logger.info('Starting remote video recorder with info {}'.format(self._req))
            try:
                self._req.command = RemoteRecordRequest.START
                res = self.service(self._req)
                if not res.ok:
                    logger.warn('Failed to start remote video recorder: {}'.format(res.message))
            except Exception as ex:
                logger.warn('Exception when starting remote video recorder: {}'.format(str(e)))

    def stop(self):
        if self.service is not None:
            try:
                res = self.service(command=RemoteRecordRequest.STOP)
                if not res.ok:
                    logger.warn('Failed to stop remote video recorder: {}'.format(res.message))
            except Exception as ex:
                logger.warn('Exception when stopping remote video recorder: {}'.format(str(e)))

_SOURCE_TYPES = collections.OrderedDict((
    ('Topic', RemoteRecordRequest.SOURCE_TOPIC),
    ('Device', RemoteRecordRequest.SOURCE_DEV)
))

class RemoteRecorderConfigFrame(tk.LabelFrame, object):
    def __init__(self, parent, initial_config):
        super(RemoteRecorderConfigFrame, self).__init__(parent, text='Remote Video Recorder')
        initial_config = initial_config.get(REMOTE_VIDEO_RECORDER_CONFIG_NAME, {})

        self.enabled_var = tk.BooleanVar(value=AVAILABLE and initial_config.get('enabled', False))
        self.enabled_checkbox = tk.Checkbutton(self, variable=self.enabled_var,
                state=tk.NORMAL if AVAILABLE else tk.DISABLED, text="Enable remote video recording",
                command=self._update_enabled)
        self.enabled_checkbox.grid(row=0, column=0, columnspan=2, sticky='nw')


        self.control_topic_var = tk.StringVar(value=initial_config.get('control_topic', '/remote_video_recorder/command'))
        self.control_topic_entry = tk.Entry(self, textvariable=self.control_topic_var)
        self.control_topic_label = tk.Label(self, text='Control topic:')
        self.control_topic_label.grid(row=1, column=0, sticky='nw')
        self.control_topic_entry.grid(row=1, column=1, sticky='new')

        self._fps_var = tk.StringVar(value=initial_config.get('fps', "30"))
        self._fps_entry = tk.Entry(self, textvariable=self._fps_var)  # todo: validate
        self._fps_label = tk.Label(self, text='FPS:')
        self._fps_label.grid(row=2, column=0, sticky='nw')
        self._fps_entry.grid(row=2, column=1, sticky='new')

        self._type_var = tk.StringVar(value=initial_config.get('source_type', _SOURCE_TYPES.keys()[0]))
        self._type_sel = tk.OptionMenu(self, self._type_var, *_SOURCE_TYPES.keys())
        self._type_label = tk.Label(self, text='Source type:')
        self._type_label.grid(row=3, column=0, sticky='nw')
        self._type_sel.grid(row=3, column=1, sticky='new')

        self._source_var = tk.StringVar(value=initial_config.get('source', ''))
        self._source_entry = tk.Entry(self, textvariable=self._source_var)
        self._source_label = tk.Label(self, text='Source:')
        self._source_label.grid(row=4, column=0, sticky='nw')
        self._source_entry.grid(row=4, column=1, sticky='new')


        self.columnconfigure(1, weight=1)
        if AVAILABLE:
            self._update_enabled()
        else:
            self._set_enabled(tk.DISABLED)

    def get_config(self):
        return { REMOTE_VIDEO_RECORDER_CONFIG_NAME: {
            'enabled': self.enabled_var.get(),
            'control_topic': self.control_topic_var.get(),
            'fps': float(self._fps_var.get()),
            'source_type': self._type_var.get(),
            'source': self._source_var.get()
        }}

    def set_state(self, state):
        if AVAILABLE: # if it's not available, we never want to enable it
            self.enabled_checkbox.configure(state=state)
            if state == tk.NORMAL:
                self._update_enabled()
            else:
                self._set_enabled(state)

    def _update_enabled(self):
        self._set_enabled(tk.NORMAL if self.enabled_var.get() else tk.DISABLED)

    def _set_enabled(self, state):
        self.control_topic_entry.configure(state=state)
        self._fps_entry.configure(state=state)
        self._type_sel.configure(state=state)
        self._source_entry.configure(state=state)
                
def get_remote_video_recorder(log_dir, config):
    if REMOTE_VIDEO_RECORDER_CONFIG_NAME in config and config[REMOTE_VIDEO_RECORDER_CONFIG_NAME]['enabled']:
        cfg = config[REMOTE_VIDEO_RECORDER_CONFIG_NAME]
        return RemoteRecorder(cfg['control_topic'],
            filename=os.path.join(os.path.basename(log_dir), 'user_video.mp4'), 
            fps=cfg['fps'],
            source_type=_SOURCE_TYPES[cfg['source_type']],
            source=cfg['source']
            )
    else:
        return None
