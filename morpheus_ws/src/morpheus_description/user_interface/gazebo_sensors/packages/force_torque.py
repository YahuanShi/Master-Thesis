import xml.etree.ElementTree as ET
import os
import numpy as np

class GazeboForceTorque:

    def __init__(self, ui, root, path):

        self.ui = ui
        self.package_path = path
        self.root_gazebo = root

    def read_force_torque(self):

        if self.ui.ft_select.currentText() == 'new ft':
            self.gazebo_sensors_template_path = os.path.join(self.package_path, 'urdf/04_templates/gazebo.template.xacro') 
            self.tree_sensors = ET.parse(self.gazebo_sensors_template_path)
            self.root_sensors = self.tree_sensors.getroot()
            self.ui.ft_add_button.setEnabled(True)
            self.ui.ft_modify_button.setEnabled(False)
            self.ui.ft_remove_button.setEnabled(False)
            root = self.root_sensors
            ft_name = 'new ft'
            ft_reference = 'new_ft_joint'
        else:
            ft = self.ui.ft_select.currentText()
            ft = ft.split()
            ft_name = ft[0]
            ft_reference = ft[2]
            root = self.root_gazebo
            self.ui.ft_add_button.setEnabled(False)
            self.ui.ft_modify_button.setEnabled(True)
            self.ui.ft_remove_button.setEnabled(True)

        self.element = root.find(".//sensor[@name='" + ft_name + "']/..[@reference='" + ft_reference + "']")
        self.ft_reference = self.element.attrib.get('reference')
        sensor_element = self.element.find('sensor')
        self.ft_name = sensor_element.attrib.get('name')
        self.ft_topic = sensor_element.find('topic').text
        self.ft_always_on = sensor_element.find('always_on').text
        self.ft_visualize = sensor_element.find('visualize').text
        self.ft_update_rate = sensor_element.find('update_rate').text
        self.ft_pose = sensor_element.find('pose').text.split()
        self.ft_x = float(self.ft_pose[0])
        self.ft_y = float(self.ft_pose[1])
        self.ft_z = float(self.ft_pose[2])
        self.ft_r = float(self.ft_pose[3])
        self.ft_p = float(self.ft_pose[4])
        self.ft_yaw = float(self.ft_pose[5])
        ft_element = sensor_element.find('force_torque')
        self.ft_direction = ft_element.find('measure_direction').text
        self.ft_frame = ft_element.find('frame').text

    def set_user_interface(self):

        ft = str(self.ft_name + ' | ' + self.ft_reference)
        self.ui.ft_select.setCurrentText(ft)
        self.ui.ft_name.setText(self.ft_name)
        self.ui.ft_topic.setText(self.ft_topic)
        self.ui.ft_update_rate.setValue(int(self.ft_update_rate))
        self.ui.ft_update_rate.setToolTip(str(self.ft_update_rate))
        self.ui.ft_x.setValue(self.ft_x)
        self.ui.ft_x.setToolTip(str(self.ft_x))
        self.ui.ft_y.setValue(self.ft_y)
        self.ui.ft_y.setToolTip(str(self.ft_y))
        self.ui.ft_z.setValue(self.ft_z)
        self.ui.ft_z.setToolTip(str(self.ft_z))
        self.ui.ft_r.setValue(np.rad2deg(self.ft_r))
        self.ui.ft_r.setToolTip(str(self.ft_r))
        self.ui.ft_p.setValue(np.rad2deg(self.ft_p))
        self.ui.ft_p.setToolTip(str(self.ft_p))
        self.ui.ft_yaw.setValue(np.rad2deg(self.ft_yaw))
        self.ui.ft_yaw.setToolTip(str(self.ft_yaw))
        
        if self.ft_always_on == "true":
            self.ui.ft_on_yes.setChecked(True)
        else:
            self.ui.ft_on_no.setChecked(True)
        if self.ft_visualize == "true":
            self.ui.ft_viz_yes.setChecked(True)
        else:
            self.ui.ft_viz_no.setChecked(True)

        self.ui.ft_direction.setCurrentText(self.ft_direction)
        self.ui.ft_frame.setCurrentText(self.ft_frame)

    def set_ft(self, root):

        self.element = root.find(".//sensor[@name='" + self.ft_name + "']/..[@reference='" + self.ft_reference + "']")
        self.element.set('reference', str(self.ui.ft_reference.currentText() + '_joint'))
        sensor_element = self.element.find('sensor')
        sensor_element.set('name', self.ui.ft_name.text())
        sensor_element.find('topic').text = self.ui.ft_topic.text()
        pose = str(self.ui.ft_x.value()) + ' ' + str(self.ui.ft_y.value()) + ' ' + str(self.ui.ft_z.value()) + ' ' + str(np.deg2rad(self.ui.ft_r.value())) + ' ' + str(np.deg2rad(self.ui.ft_p.value())) + ' ' + str(np.deg2rad(self.ui.ft_yaw.value()))
        sensor_element.find('pose').text = pose
        if self.ui.ft_on_yes.isChecked():
            sensor_element.find('always_on').text = "True"
        else:
            sensor_element.find('always_on').text = "False"

        if self.ui.ft_viz_yes.isChecked():
            sensor_element.find('visualize').text = "True"
        else:
            sensor_element.find('visualize').text = "False"
        sensor_element.find('update_rate').text = self.ui.ft_update_rate.text()

        ft_element = sensor_element.find('force_torque')
        ft_element.find('measure_direction').text = self.ui.ft_direction.currentText()
        ft_element.find('frame').text = self.ui.ft_frame.currentText()

        return self.element