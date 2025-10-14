import xml.etree.ElementTree as ET
import os
import numpy as np

class GazeboIMU:

    def __init__(self, ui, root, path):
        
        self.ui = ui
        self.package_path = path
        self.root_gazebo = root

    def read_imu(self):

        if self.ui.imu_select.currentText() == 'new imu':
            self.gazebo_sensors_template_path = os.path.join(self.package_path, 'urdf/04_templates/gazebo.template.xacro') 
            self.tree_sensors = ET.parse(self.gazebo_sensors_template_path)
            self.root_sensors = self.tree_sensors.getroot()
            self.ui.imu_add_button.setEnabled(True)
            self.ui.imu_modify_button.setEnabled(False)
            self.ui.imu_remove_button.setEnabled(False)
            root = self.root_sensors
            imu_name = 'new imu'
            imu_reference = 'new_imu_link'
        else:
            imu = self.ui.imu_select.currentText()
            imu = imu.split()
            imu_name = imu[0]
            imu_reference = imu[2]
            root = self.root_gazebo
            self.ui.imu_add_button.setEnabled(False)
            self.ui.imu_modify_button.setEnabled(True)
            self.ui.imu_remove_button.setEnabled(True)

        self.element = root.find(".//sensor[@name='" + imu_name + "']/..[@reference='" + imu_reference + "']")
        self.imu_reference = self.element.attrib.get('reference')
        sensor_element = self.element.find('sensor')
        self.imu_name = sensor_element.attrib.get('name')
        self.imu_topic = sensor_element.find('topic').text
        self.imu_always_on = sensor_element.find('always_on').text
        self.imu_visualize = sensor_element.find('visualize').text
        self.imu_update_rate = sensor_element.find('update_rate').text
        self.imu_pose = sensor_element.find('pose').text.split()
        self.imu_x = float(self.imu_pose[0])
        self.imu_y = float(self.imu_pose[1])
        self.imu_z = float(self.imu_pose[2])
        self.imu_r = float(self.imu_pose[3])
        self.imu_p = float(self.imu_pose[4])
        self.imu_yaw = float(self.imu_pose[5])

    def set_imu(self, root):

        self.element = root.find(".//sensor[@name='" + self.imu_name + "']/..[@reference='" + self.imu_reference + "']")
        self.element.set('reference', str(self.ui.imu_reference.currentText() + '_link'))
        sensor_element = self.element.find('sensor')
        sensor_element.set('name', self.ui.imu_name.text())
        sensor_element.find('topic').text = self.ui.imu_topic.text()
        pose = str(self.ui.imu_x.value()) + ' ' + str(self.ui.imu_y.value()) + ' ' + str(self.ui.imu_z.value()) + ' ' + str(np.deg2rad(self.ui.imu_r.value())) + ' ' + str(np.deg2rad(self.ui.imu_p.value())) + ' ' + str(np.deg2rad(self.ui.imu_yaw.value()))
        sensor_element.find('pose').text = pose
        if self.ui.imu_on_yes.isChecked():
            sensor_element.find('always_on').text = "True"
        else:
            sensor_element.find('always_on').text = "False"

        if self.ui.imu_viz_yes.isChecked():
            sensor_element.find('visualize').text = "True"
        else:
            sensor_element.find('visualize').text = "False"
        sensor_element.find('update_rate').text = self.ui.imu_update_rate.text()

        return self.element
    
    def set_user_interface(self):

        imu = str(self.imu_name + ' | ' + self.imu_reference)
        self.ui.imu_select.setCurrentText(imu)
        self.ui.imu_name.setText(self.imu_name)
        self.ui.imu_topic.setText(self.imu_topic)
        self.ui.imu_update_rate.setValue(int(self.imu_update_rate))
        self.ui.imu_update_rate.setToolTip(str(self.imu_update_rate))
        self.ui.imu_x.setValue(self.imu_x)
        self.ui.imu_x.setToolTip(str(self.imu_x))
        self.ui.imu_y.setValue(self.imu_y)
        self.ui.imu_y.setToolTip(str(self.imu_y))
        self.ui.imu_z.setValue(self.imu_z)
        self.ui.imu_z.setToolTip(str(self.imu_z))
        self.ui.imu_r.setValue(np.rad2deg(self.imu_r))
        self.ui.imu_r.setToolTip(str(self.imu_r))
        self.ui.imu_p.setValue(np.rad2deg(self.imu_p))
        self.ui.imu_p.setToolTip(str(self.imu_p))
        self.ui.imu_yaw.setValue(np.rad2deg(self.imu_yaw))
        self.ui.imu_yaw.setToolTip(str(self.imu_yaw))
        if self.imu_always_on == "true":
            self.ui.imu_on_yes.setChecked(True)
        else:
            self.ui.imu_on_no.setChecked(True)
        if self.imu_visualize == "true":
            self.ui.imu_viz_yes.setChecked(True)
        else:
            self.ui.imu_viz_no.setChecked(True)