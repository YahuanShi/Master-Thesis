import xml.etree.ElementTree as ET
import os
import numpy as np

class GazeboCamera:

    def __init__(self, ui, root, path):

        self.ui = ui
        self.package_path = path
        self.root_gazebo = root
    
    def read_camera(self):

        if self.ui.camera_select.currentText() == 'new camera':
            self.gazebo_sensors_template_path = os.path.join(self.package_path, 'urdf/04_templates/gazebo.template.xacro') 
            self.tree_sensors = ET.parse(self.gazebo_sensors_template_path)
            self.root_sensors = self.tree_sensors.getroot()
            self.ui.camera_add_button.setEnabled(True)
            self.ui.camera_modify_button.setEnabled(False)
            self.ui.camera_remove_button.setEnabled(False)
            root = self.root_sensors
            camera_name = 'new camera'
            camera_reference = 'new_camera_link'
        else:
            camera = self.ui.camera_select.currentText()
            camera = camera.split()
            camera_name = camera[0]
            camera_reference = camera[2]
            root = self.root_gazebo
            self.ui.camera_add_button.setEnabled(False)
            self.ui.camera_modify_button.setEnabled(True)
            self.ui.camera_remove_button.setEnabled(True)

        self.element = root.find(".//sensor[@name='" + camera_name + "']/..[@reference='" + camera_reference + "']")
        self.camera_reference = self.element.attrib.get('reference')
        sensor_element = self.element.find('sensor')
        self.camera_name = sensor_element.attrib.get('name')
        self.camera_type = sensor_element.attrib.get('type')
        self.camera_topic = sensor_element.find('topic').text
        self.camera_always_on = sensor_element.find('always_on').text
        self.camera_visualize = sensor_element.find('visualize').text
        self.camera_update_rate = sensor_element.find('update_rate').text
        self.camera_pose = sensor_element.find('pose').text.split()
        self.camera_x = float(self.camera_pose[0])
        self.camera_y = float(self.camera_pose[1])
        self.camera_z = float(self.camera_pose[2])
        self.camera_r = float(self.camera_pose[3])
        self.camera_p = float(self.camera_pose[4])
        self.camera_yaw = float(self.camera_pose[5])

        camera_element = sensor_element.find('camera')
        self.camera_horizontal_fov = camera_element.find('horizontal_fov').text

        image_element = camera_element.find('image')
        self.camera_image_width = image_element.find('width').text
        self.camera_image_height = image_element.find('height').text

        clip_element = camera_element.find('clip')
        self.camera_clip_near = clip_element.find('near').text
        self.camera_clip_far = clip_element.find('far').text

    def set_user_interface(self):

        camera = str(self.camera_name + ' | ' + self.camera_reference)
        self.ui.camera_select.setCurrentText(camera)
        self.ui.camera_name.setText(self.camera_name)
        self.ui.camera_reference.setCurrentText(self.camera_reference.replace('_link',''))
        self.ui.camera_type.setCurrentText(self.camera_type)
        self.ui.camera_update_rate.setValue(int(self.camera_update_rate))
        self.ui.camera_update_rate.setToolTip(str(self.camera_update_rate))
        self.ui.camera_topic.setText(self.camera_topic)
        self.ui.camera_x.setValue(self.camera_x)
        self.ui.camera_x.setToolTip(str(self.camera_x))
        self.ui.camera_y.setValue(self.camera_y)
        self.ui.camera_y.setToolTip(str(self.camera_y))
        self.ui.camera_z.setValue(self.camera_z)
        self.ui.camera_z.setToolTip(str(self.camera_z))
        self.ui.camera_r.setValue(np.rad2deg(self.camera_r))
        self.ui.camera_r.setToolTip(str(self.camera_r))
        self.ui.camera_p.setValue(np.rad2deg(self.camera_p))
        self.ui.camera_p.setToolTip(str(self.camera_p))
        self.ui.camera_yaw.setValue(np.rad2deg(self.camera_yaw))
        self.ui.camera_yaw.setToolTip(str(self.camera_yaw))
        self.ui.camera_fov.setValue(np.rad2deg(float(self.camera_horizontal_fov)))
        self.ui.camera_fov.setToolTip(str(self.camera_horizontal_fov))
        self.ui.camera_image_height.setValue(float(self.camera_image_height))
        self.ui.camera_image_height.setToolTip(str(self.camera_image_height))
        self.ui.camera_image_width.setValue(float(self.camera_image_width))
        self.ui.camera_image_width.setToolTip(str(self.camera_image_width))
        self.ui.camera_clip_near.setValue(float(self.camera_clip_near))
        self.ui.camera_clip_near.setToolTip(str(self.camera_clip_near))
        self.ui.camera_clip_far.setValue(float(self.camera_clip_far))
        self.ui.camera_clip_far.setToolTip(str(self.camera_clip_far))
        if self.camera_always_on == "true":
            self.ui.camera_on_yes.setChecked(True)
        else:
            self.ui.camera_on_no.setChecked(True)
        if self.camera_visualize == "true":
            self.ui.camera_viz_yes.setChecked(True)
        else:
            self.ui.camera_viz_no.setChecked(True)

    def set_camera(self, root):

        self.element = root.find(".//sensor[@name='" + self.camera_name + "']/..[@reference='" + self.camera_reference + "']")
        self.element.set('reference', str(self.ui.camera_reference.currentText() + '_link'))
        sensor_element = self.element.find('sensor')
        sensor_element.set('name',self.ui.camera_name.text())
        sensor_element.set('type',self.ui.camera_type.currentText())
        pose = str(self.ui.camera_x.value()) + ' ' + str(self.ui.camera_y.value()) + ' ' + str(self.ui.camera_z.value()) + ' ' + str(np.deg2rad(self.ui.camera_r.value())) + ' ' + str(np.deg2rad(self.ui.camera_p.value())) + ' ' + str(np.deg2rad(self.ui.camera_yaw.value()))
        sensor_element.find('pose').text = pose
    
        sensor_element.find('topic').text = self.ui.camera_topic.text()
        if self.ui.camera_on_yes.isChecked():
            sensor_element.find('always_on').text = "True"
        else:
            sensor_element.find('always_on').text = "False"

        if self.ui.camera_viz_yes.isChecked():
            sensor_element.find('visualize').text = "True"
        else:
            sensor_element.find('visualize').text = "False"
        sensor_element.find('update_rate').text = self.ui.camera_update_rate.text()

        camera_element = sensor_element.find('camera')
        camera_element.find('horizontal_fov').text = str(np.deg2rad(self.ui.camera_fov.value()))

        image_element = camera_element.find('image')
        image_element.find('width').text = self.ui.camera_image_width.text()
        image_element.find('height').text = self.ui.camera_image_height.text()

        clip_element = camera_element.find('clip')
        clip_element.find('near').text = self.ui.camera_clip_near.text()
        clip_element.find('far').text = self.ui.camera_clip_far.text()

        return self.element