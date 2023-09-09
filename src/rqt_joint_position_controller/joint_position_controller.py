#!/usr/bin/env python

# Copyright (C) 2014, PAL Robotics S.L.
# Copyright (C) 2023, Tony Le (tonyle98@outlook.com) [Modifications]
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
# * Neither the name of PAL Robotics S.L. nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import print_function
import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer, Signal
from python_qt_binding.QtWidgets import QWidget, QFormLayout

from controller_manager_msgs.utils\
    import ControllerLister, ControllerManagerLister
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState

from .double_editor import DoubleEditor
from .joint_limits_urdf import get_joint_limits
from .update_combo import update_combo


class JointPositionController(Plugin):
    _cmd_pub_freq = 10.0  # Hz
    _widget_update_freq = 30.0  # Hz
    _ctrlrs_update_freq = 1  # Hz

    jointStateChanged = Signal([dict])

    def __init__(self, context):
        super(JointPositionController, self).__init__(context)
        self.setObjectName('JointPositionController')

        # Create QWidget and extend it with all the attributes and children
        # from the UI file
        self._widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_joint_position_controller'),
                               'resource',
                               'joint_position_controller.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('JointPositionControllerUi')
        ns = rospy.get_namespace()[1:-1]
        self._widget.controller_group.setTitle('ns: ' + ns)

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() +
                                        (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # Initialize members
        self._jpc_name = []  # Name of selected joint position controller
        self._cm_ns = []  # Namespace of the selected controller manager
        self._joint_pos = {}  # name->pos map for joints of selected controller
        self._joint_names = []  # Ordered list of selected controller joints
        self._robot_joint_limits = {} # Lazily evaluated on first use

        # Timer for sending commands to active controller
        self._update_cmd_timer = QTimer(self)
        self._update_cmd_timer.setInterval(int(1000.0 / self._cmd_pub_freq))
        self._update_cmd_timer.timeout.connect(self._update_cmd_cb)

        # Timer for updating the joint widgets from the controller state
        self._update_act_pos_timer = QTimer(self)
        self._update_act_pos_timer.setInterval(int(1000.0 /
                                               self._widget_update_freq))
        self._update_act_pos_timer.timeout.connect(self._update_joint_widgets)

        # Timer for controller manager updates
        self._list_cm = ControllerManagerLister()
        self._update_cm_list_timer = QTimer(self)
        self._update_cm_list_timer.setInterval(int(1000.0 /
                                               self._ctrlrs_update_freq))
        self._update_cm_list_timer.timeout.connect(self._update_cm_list)
        self._update_cm_list_timer.start()

        # Timer for running controller updates
        self._update_jpc_list_timer = QTimer(self)
        self._update_jpc_list_timer.setInterval(int(1000.0 /
                                                self._ctrlrs_update_freq))
        self._update_jpc_list_timer.timeout.connect(self._update_jpc_list)
        self._update_jpc_list_timer.start()

        # Signal connections
        w = self._widget
        w.enable_button.toggled.connect(self._on_jpc_enabled)
        w.jpc_combo.currentIndexChanged[str].connect(self._on_jpc_change)
        w.cm_combo.currentIndexChanged[str].connect(self._on_cm_change)

        self._cmd_pub = None    # Controller command publisher

        self._joint_state_sub = None
        self._joint_state = {}

        self._list_controllers = None
        self._controller_type = None

    def shutdown_plugin(self):
        self._update_cmd_timer.stop()
        self._update_act_pos_timer.stop()
        self._update_cm_list_timer.stop()
        self._update_jpc_list_timer.stop()
        self._unregister_state_sub()
        self._unregister_cmd_pub()

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('cm_ns', self._cm_ns)
        instance_settings.set_value('jpc_name', self._jpc_name)

    def restore_settings(self, plugin_settings, instance_settings):
        # Restore last session's controller_manager, if present
        self._update_cm_list()
        cm_ns = instance_settings.value('cm_ns')
        cm_combo = self._widget.cm_combo
        cm_list = [cm_combo.itemText(i) for i in range(cm_combo.count())]
        try:
            idx = cm_list.index(cm_ns)
            cm_combo.setCurrentIndex(idx)
            # Resore last session's controller, if running
            self._update_jpc_list()
            jpc_name = instance_settings.value('jpc_name')
            jpc_combo = self._widget.jpc_combo
            jpc_list = [jpc_combo.itemText(i) for i in range(jpc_combo.count())]
            try:
                idx = jpc_list.index(jpc_name)
                jpc_combo.setCurrentIndex(idx)
            except (ValueError):
                pass
        except (ValueError):
            pass

    def _update_cm_list(self):
        update_combo(self._widget.cm_combo, self._list_cm())

    def _update_jpc_list(self):
        # Clear controller list if no controller information is available
        if not self._list_controllers:
            self._widget.jpc_combo.clear()
            return

        # List of running controllers with a valid joint limits specification
        # for _all_ their joints
        running_jpc = self._running_jpc_info()
        if running_jpc and not self._robot_joint_limits:
            self._robot_joint_limits = get_joint_limits()  # Lazy evaluation
        valid_jpc = []
        for jpc_info in running_jpc:
            has_limits = all(name in self._robot_joint_limits
                             for name in _jpc_joint_names(jpc_info))
            if has_limits:
                valid_jpc.append(jpc_info)
        valid_jpc_names = [data.name for data in valid_jpc]

        # Update widget
        update_combo(self._widget.jpc_combo, sorted(valid_jpc_names))

    def _on_joint_state_change(self, actual_pos):
        assert(len(actual_pos) == len(self._joint_pos))
        for name in actual_pos.keys():
            self._joint_pos[name]['position'] = actual_pos[name]

    def _on_cm_change(self, cm_ns):
        self._cm_ns = cm_ns
        if cm_ns:
            self._list_controllers = ControllerLister(cm_ns)
            # NOTE: Clear below is important, as different controller managers
            # might have controllers with the same name but different
            # configurations. Clearing forces controller re-discovery
            self._widget.jpc_combo.clear()
            self._update_jpc_list()
        else:
            self._list_controllers = None

    def _on_jpc_change(self, jpc_name):
        self._unload_jpc()
        self._jpc_name = jpc_name
        if self._jpc_name:
            self._load_jpc()

    def _on_jpc_enabled(self, val):
        # Don't allow enabling if there are no controllers selected
        if not self._jpc_name:
            self._widget.enable_button.setChecked(False)
            return

        # Enable/disable joint displays
        for joint_widget in self._joint_widgets():
            joint_widget.setEnabled(val)

        if val:
            # Widgets send desired position commands to controller
            self._update_act_pos_timer.stop()
            self._update_cmd_timer.start()
        else:
            # Controller updates widgets with actual position
            self._update_cmd_timer.stop()
            self._update_act_pos_timer.start()

    def _load_jpc(self):
        # Initialize joint data corresponding to selected controller
        running_jpc = self._running_jpc_info()
        self._joint_names = next(_jpc_joint_names(x) for x in running_jpc
                                 if x.name == self._jpc_name)
        
        if len(self._joint_names) == 1:
            self._controller_type = 'single_joint'
        else:
            self._controller_type = 'multi_joint'

        for name in self._joint_names:
            self._joint_pos[name] = {}

        # Update joint display
        try:
            layout = self._widget.joint_group.layout()
            for name in self._joint_names:
                limits = self._robot_joint_limits[name]
                joint_widget = DoubleEditor(limits['min_position'],
                                            limits['max_position'])
                layout.addRow(name, joint_widget)
                # NOTE: Using partial instead of a lambda because lambdas
                # "will not evaluate/look up the argument values before it is
                # effectively called, breaking situations like using a loop
                # variable inside it"
                from functools import partial
                par = partial(self._update_single_cmd_cb, name=name)
                joint_widget.valueChanged.connect(par)

                joint_pos = self._joint_state.get(name, 0.0)  # Use current joint state or default to 0
                self._joint_pos[name] = {'position': joint_pos}

        except:
            from sys import exc_info
            print('Unexpected error:', exc_info()[0])

        # Enter monitor mode (sending commands disabled)
        self._on_jpc_enabled(False)

        # Setup ROS interfaces
        jpc_ns = _resolve_controller_ns(self._cm_ns, self._jpc_name)
        cmd_topic = jpc_ns + '/command'

        if self._controller_type == 'single_joint':
            cmd_msg_type = Float64
        else:
            cmd_msg_type = Float64MultiArray  
        
        self._cmd_pub = rospy.Publisher(cmd_topic, cmd_msg_type, queue_size=1)

        joint_state_topic = self._cm_ns.replace('/controller_manager', '') + '/joint_states'
        self._joint_state_sub = rospy.Subscriber(joint_state_topic, JointState, self._joint_state_cb, queue_size=1)

        # Start updating the joint positions
        self.jointStateChanged.connect(self._on_joint_state_change)

    def _unload_jpc(self):
        # Stop updating the joint positions
        try:
            self.jointStateChanged.disconnect(self._on_joint_state_change)
        except:
            pass

        # Reset ROS interfaces
        self._unregister_joint_state_sub()
        self._unregister_cmd_pub()

        # Clear joint widgets
        layout = self._widget.joint_group.layout()
        if layout is not None:
            while layout.count():
                layout.takeAt(0).widget().deleteLater()
            # Delete existing layout by reparenting to temporary
            QWidget().setLayout(layout)
        self._widget.joint_group.setLayout(QFormLayout())

        # Reset joint data
        self._joint_names = []
        self._joint_pos = {}

        # Enforce monitor mode (sending commands disabled)
        self._widget.enable_button.setChecked(False)
        

    def _running_jpc_info(self):
        from controller_manager_msgs.utils import filter_by_type, filter_by_state

        controller_list = self._list_controllers()
        jpc_list = filter_by_type(controller_list, 'JointPositionController', match_substring=True)
        jgpc_list = filter_by_type(controller_list, 'JointGroupPositionController', match_substring=True)
        
        # Combine both lists
        combined_list = jpc_list + jgpc_list
        running_combined_list = filter_by_state(combined_list, 'running')
        return running_combined_list


    def _unregister_cmd_pub(self):
        if self._cmd_pub is not None:
            self._cmd_pub.unregister()
            self._cmd_pub = None

    def _unregister_joint_state_sub(self):
        if self._joint_state_sub is not None:
            self._joint_state_sub.unregister()
            self._joint_state_sub = None

    def _joint_state_cb(self, msg):
        actual_pos = {}
        for name in self._joint_names:
            for idx, joint in enumerate(msg.name):
                if name == joint:
                    actual_pos[name] = msg.position[idx]
                    break
        self.jointStateChanged.emit(actual_pos)

    def _update_single_cmd_cb(self, val, name):
        self._joint_pos[name]['command'] = val

    def _update_cmd_cb(self):
        if self._controller_type == 'single_joint':
            for name in self._joint_names:
                cmd = self._joint_pos[name].get('command', None) 
                if cmd is not None:
                    self._cmd_pub.publish(Float64(cmd))
        else:
            cmd_array = Float64MultiArray()
            cmd_array.data = [self._joint_pos[name].get('command', 0.0) for name in self._joint_names]
            self._cmd_pub.publish(cmd_array)


    def _update_joint_widgets(self):
        joint_widgets = self._joint_widgets()
        for id in range(len(joint_widgets)):
            joint_name = self._joint_names[id]
            try:
                joint_pos = self._joint_pos[joint_name]['position']
                joint_widgets[id].setValue(joint_pos)
            except (KeyError):
                pass  # Can happen when first connected to controller

    def _joint_widgets(self):
        widgets = []
        layout = self._widget.joint_group.layout()
        for row_id in range(layout.rowCount()):
            widgets.append(layout.itemAt(row_id,
                                         QFormLayout.FieldRole).widget())
        return widgets

def _jpc_joint_names(jpc_info):
    # NOTE: We assume that there is at least one hardware interface that
    # claims resources (there should be), and the resource list is fetched
    # from the first available interface
    return jpc_info.claimed_resources[0].resources

def _resolve_controller_ns(cm_ns, controller_name):
    assert(controller_name)
    ns = cm_ns.rsplit('/', 1)[0]
    if ns != '/':
        ns += '/'
    ns += controller_name
    return ns
