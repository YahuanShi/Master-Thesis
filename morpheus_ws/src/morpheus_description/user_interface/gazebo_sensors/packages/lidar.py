import xml.etree.ElementTree as ET
import os
import numpy as np

class GazeboLidar:
    def __init__(self, ui, root, path):

        self.ui = ui
        self.package_path = path
        self.root_gazebo = root

    def read_lidar(self):

        if self.ui.lidar_select.currentText() == 'new lidar':
            self.gazebo_sensors_template_path = os.path.join(self.package_path, 'urdf/04_templates/gazebo.template.xacro') 
            self.tree_sensors = ET.parse(self.gazebo_sensors_template_path)
            self.root_sensors = self.tree_sensors.getroot()
            self.ui.lidar_add_button.setEnabled(True)
            self.ui.lidar_modify_button.setEnabled(False)
            self.ui.lidar_remove_button.setEnabled(False)
            root = self.root_sensors
            lidar_name = 'new lidar'
            lidar_reference = 'new_lidar_link'
        else:
            lidar = self.ui.lidar_select.currentText()
            lidar = lidar.split()
            lidar_name = lidar[0]
            lidar_reference = lidar[2]
            root = self.root_gazebo
            self.ui.lidar_add_button.setEnabled(False)
            self.ui.lidar_modify_button.setEnabled(True)
            self.ui.lidar_remove_button.setEnabled(True)

        self.element = root.find(".//sensor[@name='" + lidar_name + "']/..[@reference='" + lidar_reference + "']")
        self.lidar_reference = self.element.attrib.get('reference')
        sensor_element = self.element.find('sensor')
        self.lidar_name = sensor_element.attrib.get('name')
        self.lidar_type = sensor_element.attrib.get('type')
        self.lidar_topic = sensor_element.find('topic').text
        self.lidar_update_rate = sensor_element.find('update_rate').text
        self.lidar_always_on = sensor_element.find('always_on').text
        self.lidar_visualize = sensor_element.find('visualize').text
        self.lidar_pose = sensor_element.find('pose').text.split()
        self.lidar_x = float(self.lidar_pose[0])
        self.lidar_y = float(self.lidar_pose[1])
        self.lidar_z = float(self.lidar_pose[2])
        self.lidar_r = float(self.lidar_pose[3])
        self.lidar_p = float(self.lidar_pose[4])
        self.lidar_yaw = float(self.lidar_pose[5])

        lidar_element = sensor_element.find('lidar')

        scan_element = lidar_element.find('scan')
        horizontal_element = scan_element.find('horizontal')
        self.lidar_samples_horizontal = horizontal_element.find('samples').text
        self.lidar_resolution_horizontal = horizontal_element.find('resolution').text
        self.lidar_min_angle_horizontal = horizontal_element.find('min_angle').text
        self.lidar_max_angle_horizontal = horizontal_element.find('max_angle').text

        vertical_element = scan_element.find('vertical')
        self.lidar_samples_vertical = vertical_element.find('samples').text
        self.lidar_resolution_vertical = vertical_element.find('resolution').text
        self.lidar_min_angle_vertical = vertical_element.find('min_angle').text
        self.lidar_max_angle_vertical = vertical_element.find('max_angle').text

        range_element = lidar_element.find('range')
        self.lidar_min_range = range_element.find('min').text
        self.lidar_max_range = range_element.find('max').text

    def set_lidar(self, root):

        self.element = root.find(".//sensor[@name='" + self.lidar_name + "']/..[@reference='" + self.lidar_reference + "']")

        self.element.set('reference', str(self.ui.lidar_reference.currentText() + '_link'))
        sensor_element = self.element.find('sensor')
        sensor_element.set('name', self.ui.lidar_name.text())
        sensor_element.set('type', self.ui.lidar_type.currentText())
        sensor_element.find('topic').text = self.ui.lidar_topic.text()
        sensor_element.find('update_rate').text = self.ui.lidar_update_rate.text()
        pose = str(self.ui.lidar_x.value()) + ' ' + str(self.ui.lidar_y.value()) + ' ' + str(self.ui.lidar_z.value()) + ' ' + str(np.deg2rad(self.ui.lidar_r.value())) + ' ' + str(np.deg2rad(self.ui.lidar_p.value())) + ' ' + str(np.deg2rad(self.ui.lidar_yaw.value()))
        sensor_element.find('pose').text = pose

        if self.ui.lidar_on_yes.isChecked():
            sensor_element.find('always_on').text = "True"
        else:
            sensor_element.find('always_on').text = "False"

        if self.ui.lidar_viz_yes.isChecked():
            sensor_element.find('visualize').text = "True"
        else:
            sensor_element.find('visualize').text = "False"

        lidar_element = sensor_element.find('lidar')

        scan_element = lidar_element.find('scan')
        horizontal_element = scan_element.find('horizontal')
        horizontal_element.find('samples').text = str(self.ui.lidar_samples_h.value())
        horizontal_element.find('resolution').text = str(self.ui.lidar_resolution_h.value())
        horizontal_element.find('min_angle').text = str(np.deg2rad(self.ui.lidar_min_angle_h.value()))
        horizontal_element.find('max_angle').text = str(np.deg2rad(self.ui.lidar_max_angle_h.value()))

        vertical_element = scan_element.find('vertical')
        vertical_element.find('samples').text = str(self.ui.lidar_samples_v.value())
        vertical_element.find('resolution').text = str(self.ui.lidar_resolution_v.value())
        vertical_element.find('min_angle').text = str(np.deg2rad(self.ui.lidar_min_angle_v.value()))
        vertical_element.find('max_angle').text = str(np.deg2rad(self.ui.lidar_max_angle_v.value()))

        range_element = lidar_element.find('range')
        range_element.find('min').text = str(self.ui.lidar_min_range.value())
        range_element.find('max').text = str(self.ui.lidar_max_range.value())

        return self.element

    def set_user_interface(self):

        lidar = str(self.lidar_name + ' | ' + self.lidar_reference)
        self.ui.lidar_select.setCurrentText(lidar)
        self.ui.lidar_name.setText(self.lidar_name)
        self.ui.lidar_type.setCurrentText(self.lidar_type)
        self.ui.lidar_update_rate.setValue(float(self.lidar_update_rate))
        self.ui.lidar_update_rate.setToolTip(str(self.lidar_update_rate))
        self.ui.lidar_topic.setText(self.lidar_topic)
        self.ui.lidar_x.setValue(self.lidar_x)
        self.ui.lidar_x.setToolTip(str(self.lidar_x))
        self.ui.lidar_y.setValue(self.lidar_y)
        self.ui.lidar_y.setToolTip(str(self.lidar_y))
        self.ui.lidar_z.setValue(self.lidar_z)
        self.ui.lidar_z.setToolTip(str(self.lidar_z))
        self.ui.lidar_r.setValue(np.rad2deg(self.lidar_r))
        self.ui.lidar_r.setToolTip(str(self.lidar_r))
        self.ui.lidar_p.setValue(np.rad2deg(self.lidar_p))
        self.ui.lidar_p.setToolTip(str(self.lidar_p))
        self.ui.lidar_yaw.setValue(np.rad2deg(self.lidar_yaw))
        self.ui.lidar_yaw.setToolTip(str(self.lidar_yaw))

        self.ui.lidar_samples_h.setValue(float(self.lidar_samples_horizontal))
        self.ui.lidar_samples_h.setToolTip(str(self.lidar_samples_horizontal))
        self.ui.lidar_resolution_h.setValue(float(self.lidar_resolution_horizontal))
        self.ui.lidar_resolution_h.setToolTip(str(self.lidar_resolution_horizontal))
        self.ui.lidar_min_angle_h.setValue(np.rad2deg(float(self.lidar_min_angle_horizontal)))
        self.ui.lidar_min_angle_h.setToolTip(str(self.lidar_min_angle_horizontal))
        self.ui.lidar_max_angle_h.setValue(np.rad2deg(float(self.lidar_max_angle_horizontal)))
        self.ui.lidar_max_angle_h.setToolTip(str(self.lidar_max_angle_horizontal))

        self.ui.lidar_samples_v.setValue(float(self.lidar_samples_vertical))
        self.ui.lidar_samples_v.setToolTip(str(self.lidar_samples_vertical))
        self.ui.lidar_resolution_v.setValue(float(self.lidar_resolution_vertical))
        self.ui.lidar_resolution_v.setToolTip(str(self.lidar_resolution_vertical))
        self.ui.lidar_min_angle_v.setValue(np.rad2deg(float(self.lidar_min_angle_vertical)))
        self.ui.lidar_min_angle_v.setToolTip(str(self.lidar_min_angle_vertical))
        self.ui.lidar_max_angle_v.setValue(np.rad2deg(float(self.lidar_max_angle_vertical)))
        self.ui.lidar_max_angle_v.setToolTip(str(self.lidar_max_angle_vertical))
        
        self.ui.lidar_min_range.setValue(float(self.lidar_min_range))
        self.ui.lidar_min_range.setToolTip(str(self.lidar_min_range))
        self.ui.lidar_max_range.setValue(float(self.lidar_max_range))
        self.ui.lidar_max_range.setToolTip(str(self.lidar_max_range))

        if self.lidar_always_on == "true":
            self.ui.lidar_on_yes.setChecked(True)
        else:
            self.ui.lidar_on_no.setChecked(True)
        if self.lidar_visualize == "true":
            self.ui.lidar_viz_yes.setChecked(True)
        else:
            self.ui.lidar_viz_no.setChecked(True)