import numpy as np
import os
import xml.etree.ElementTree as ET

class RobotDescriptionXacro:

    def __init__(self, ui, path):

        self.ui = ui
        self.xacro_tag = '{http://www.ros.org/wiki/xacro}'
        self.package_path = path
        self.package_name = self.package_path.split('/')[-1]
        self.name_changed = False

        self.ros2_control_path = os.path.join(self.package_path, 'urdf/03_ros2_control/ros2_control.xacro')
        self.tree_ros2control = ET.parse(self.ros2_control_path)
        self.root_ros2control = self.tree_ros2control.getroot()  

    def read_user_interface(self):

        # Attributes
        self.name = self.ui.name_line.text()
        self.mass = str(self.ui.mass_line.value())
        self.joint_type = self.ui.comboBox_joints.currentText()
        self.parent = self.ui.comboBox_parent.currentText()
        self.mimic = self.ui.checkBox_mimic.isChecked()
        if self.mimic == True:
            self.mimic = "1"
        else:
            self.mimic = "0"
            
        self.empty = self.ui.checkBox_base.isChecked()
        if self.empty == True:
            self.empty = "1"
        else:
            self.empty = "0"
            
        # Visual Tag
        self.link_viz_x = str(self.ui.link_origin_x_viz.value())
        self.link_viz_y = str(self.ui.link_origin_y_viz.value())
        self.link_viz_z = str(self.ui.link_origin_z_viz.value())
        self.link_viz_xyz = self.link_viz_x + " " + self.link_viz_y + " " + self.link_viz_z

        self.link_viz_roll  = str(np.deg2rad(self.ui.link_origin_r_viz.value()))
        self.link_viz_pitch = str(np.deg2rad(self.ui.link_origin_p_viz.value()))
        self.link_viz_yaw   = str(np.deg2rad(self.ui.link_origin_yaw_viz.value()))
        self.link_viz_rpy = self.link_viz_roll + " " + self.link_viz_pitch + " " + self.link_viz_yaw

        if self.ui.radioButton_box_viz.isChecked():
            self.new_tag = 'box'
            self.new_attr_name = 'size'
            self.link_viz_box_x = str(self.ui.box_x_viz.value())
            self.link_viz_box_y = str(self.ui.box_y_viz.value())
            self.link_viz_box_z = str(self.ui.box_z_viz.value())
            self.link_viz_box = self.link_viz_box_x + " " + self.link_viz_box_y + " " + self.link_viz_box_z
        elif self.ui.radioButton_sphere_viz.isChecked():
            self.new_tag = 'sphere'
            self.new_attr_name = 'radius'
            self.link_viz_sph_r = str(self.ui.radius_sph_viz.value())
        elif self.ui.radioButton_cylinder_viz.isChecked():
            self.new_tag = 'cylinder'
            self.new_attr_name_r = 'radius'
            self.new_attr_name_l = 'length'
            self.link_viz_cyl_r = str(self.ui.radius_cyl_viz.value())
            self.link_viz_cyl_l = str(self.ui.length_cyl_viz.value())
        elif self.ui.radioButton_mesh_viz.isChecked():
            file = self.ui.viz_mesh_line.text()
            self.new_tag = 'mesh'
            self.new_attr_name = 'filename'
            if self.package_path in file:
                file = file.replace(self.package_path, 'package://' + self.package_name)
                self.link_viz_mesh = file
            elif "package://" in file or "file://" in file:
                self.link_viz_mesh = file
            else:
                self.link_viz_mesh = "file://" + file

        red = self.ui.toolButton_color.palette().base().color().getRgb()[0]/255
        green = self.ui.toolButton_color.palette().base().color().getRgb()[1]/255
        blue = self.ui.toolButton_color.palette().base().color().getRgb()[2]/255
        self.rgb = str(red) + " " + str(green) + " " + str(blue) + " 1.0"

        # Collision Tag
        self.link_col_x = str(self.ui.link_origin_x_col.value())
        self.link_col_y = str(self.ui.link_origin_y_col.value())
        self.link_col_z = str(self.ui.link_origin_z_col.value())
        self.link_col_xyz = self.link_col_x + " " + self.link_col_y + " " + self.link_col_z


        self.link_col_roll  = str(round(np.deg2rad(self.ui.link_origin_r_col.value()),3))
        self.link_col_pitch = str(np.deg2rad(self.ui.link_origin_p_col.value()))
        self.link_col_yaw   = str(np.deg2rad(self.ui.link_origin_yaw_col.value()))
        self.link_col_rpy = self.link_col_roll + " " + self.link_col_pitch + " " + self.link_col_yaw

        if self.ui.radioButton_box_col.isChecked():
            self.new_tag_col = 'box'
            self.new_attr_name_col = 'size'
            self.link_col_box_x = str(self.ui.box_x_col.value())
            self.link_col_box_y = str(self.ui.box_y_col.value())
            self.link_col_box_z = str(self.ui.box_z_col.value())
            self.link_col_box = self.link_col_box_x + " " + self.link_col_box_y + " " + self.link_col_box_z
        elif self.ui.radioButton_sphere_col.isChecked():
            self.new_tag_col = 'sphere'
            self.new_attr_name_col = 'radius'
            self.link_col_sph_r = str(self.ui.radius_sph_col.value())
        elif self.ui.radioButton_cylinder_col.isChecked():
            self.new_tag_col = 'cylinder'
            self.new_attr_name_r_col = 'radius'
            self.new_attr_name_l_col = 'length'
            self.link_col_cyl_r = str(self.ui.radius_cyl_col.value())
            self.link_col_cyl_l = str(self.ui.length_cyl_col.value())
        elif self.ui.radioButton_mesh_col.isChecked():
            file = self.ui.col_mesh_line.text()
            self.new_tag_col = 'mesh'
            self.new_attr_name_col = 'filename'
            if self.package_path in file:
                file = file.replace(self.package_path, 'package://' + self.package_name)
                self.link_col_mesh = file
            elif "package://" in file or "file://" in file:
                self.link_col_mesh = file
            else:
                self.link_col_mesh = "file://" + file
        
        # Inertial Tag
        self.link_com_x = str(self.ui.link_cof_x.value())
        self.link_com_y = str(self.ui.link_cof_y.value())
        self.link_com_z = str(self.ui.link_cof_z.value())
        self.link_com = self.link_com_x + " " + self.link_com_y + " " + self.link_com_z
        
        self.link_xx = str(self.ui.link_moi_xx.value())
        self.link_yy = str(self.ui.link_moi_yy.value())
        self.link_zz = str(self.ui.link_moi_zz.value())
        self.link_xy = str(self.ui.link_moi_xy.value())
        self.link_xz = str(self.ui.link_moi_xz.value())
        self.link_yz = str(self.ui.link_moi_yz.value())

        self.joint_x = str(self.ui.joint_origin_x.value())
        self.joint_y = str(self.ui.joint_origin_y.value())
        self.joint_z = str(self.ui.joint_origin_z.value())
        self.joint_xyz = self.joint_x + " " + self.joint_y + " " + self.joint_z           
        
        self.joint_roll = str(np.deg2rad(self.ui.joint_origin_r.value()))
        self.joint_pitch = str(np.deg2rad(self.ui.joint_origin_p.value()))
        self.joint_yaw = str(np.deg2rad(self.ui.joint_origin_yaw.value()))
        self.joint_rpy = self.joint_roll + " " + self.joint_pitch + " " + self.joint_yaw     
        
        self.joint_aor_x = str(self.ui.joint_aor_x.value())
        self.joint_aor_y = str(self.ui.joint_aor_y.value())
        self.joint_aor_z = str(self.ui.joint_aor_z.value()) 
        self.joint_aor =  self.joint_aor_x + " " + self.joint_aor_y + " " + self.joint_aor_z  
        
        self.joint_effort = str(self.ui.joint_limit_effort.value())
        self.joint_velocity = str(self.ui.joint_limit_vel.value())
        self.joint_lower = str(np.deg2rad(self.ui.joint_limit_lower.value()))
        self.joint_upper = str(np.deg2rad(self.ui.joint_limit_upper.value()))

        self.joint_damping = str(self.ui.joint_damping.value())
        self.joint_friction = str(self.ui.joint_friction.value())

        self.joint_multiplier = str(self.ui.joint_mimic_mult.value())
        self.joint_offset = str(self.ui.joint_mimic_offset.value())
        self.joint_mimic_joint = self.ui.comboBox_mimic.currentText()

    def set_xacro(self, root, name_attr, macro):

        for self.element in root.iter(self.xacro_tag + macro):

            if self.element.attrib.get('name') == name_attr: 

                # Part Attributes
                self.element.set('name', self.name)
                self.old_name = self.ui.comboBox.currentText()

                if self.name != self.old_name and name_attr != 'new link':
                    
                    index = self.ui.comboBox.findText(self.old_name)
                    self.ui.comboBox.removeItem(index)
                    self.ui.comboBox.insertItem(index, self.name)
                    self.ui.comboBox.setCurrentText(self.name)

                    for self.element_parent in root.iter(self.xacro_tag + 'part'):
                        if self.element_parent.attrib.get('parent') == self.old_name:
                            self.element_parent.set('parent', self.name)
                    
                    for self.element_temp in root.iter(self.xacro_tag + 'part'):
                        self.element_mimic = self.element_temp.find(self.xacro_tag + 'joint_mimic')
                        if self.element_mimic.attrib.get('joint') == self.old_name:
                            self.element_mimic.set('joint', self.name)

                    self.name_changed = True

                    # TODO: wie wirkt sich folgendes auf mehrere Cameras und ihre Refs aus
                    self.update_camera_reference()
                    self.update_lidar_reference()
                    self.update_imu_reference()
                    self.update_force_torque_reference()

                    self.update_ros2_control()
                    
                    self.name_changed = False

                self.element.set('mass', self.mass)

                if macro == 'part':
                    self.element.set('type', self.joint_type)
                    self.element.set('parent', self.parent)
                    self.element.set('mimic', self.mimic)
                else:
                    self.element.set('empty', self.empty)

                if self.new_tag == 'mesh' and '.dae' in self.link_viz_mesh.lower():
                    self.element.set('color', '0')
                else:
                    self.element.set('color','1')

                # Link Elements
                link_viz_origin = self.element.find(self.xacro_tag + 'link_viz_origin')
                link_viz_origin.set('xyz', self.link_viz_xyz)
                link_viz_origin.set('rpy', self.link_viz_rpy)

                link_viz_shape = self.element.find(self.xacro_tag + 'link_viz_shape')
                self.shape_type_viz = link_viz_shape[0].tag
                if self.shape_type_viz != 'cylinder':
                    if self.new_tag == 'cylinder':
                        link_viz_shape[0].tag = self.new_tag
                        old_attribute_name = list(link_viz_shape[0].attrib.keys())[0]
                        link_viz_shape[0].attrib[self.new_attr_name_r] = link_viz_shape[0].attrib.pop(old_attribute_name)
                        link_viz_shape[0].set(self.new_attr_name_r, self.link_viz_cyl_r)
                        link_viz_shape[0].set(self.new_attr_name_l, self.link_viz_cyl_l)
                    else:
                        link_viz_shape[0].tag = self.new_tag
                        old_attribute_name = list(link_viz_shape[0].attrib.keys())[0]
                        link_viz_shape[0].attrib[self.new_attr_name] = link_viz_shape[0].attrib.pop(old_attribute_name)
                        if self.new_tag == 'mesh':
                            link_viz_shape[0].set(self.new_attr_name, self.link_viz_mesh)
                        elif self.new_tag == 'box':
                            link_viz_shape[0].set(self.new_attr_name, self.link_viz_box)
                        elif self.new_tag == 'sphere':
                            link_viz_shape[0].set(self.new_attr_name, self.link_viz_sph_r)
                else:
                    if self.new_tag == 'cylinder':
                        link_viz_shape[0].set(self.new_attr_name_r, self.link_viz_cyl_r)
                        link_viz_shape[0].set(self.new_attr_name_l, self.link_viz_cyl_l)
                    else:
                        link_viz_shape[0].tag = self.new_tag
                        old_attribute_name = list(link_viz_shape[0].attrib.keys())[0]
                        link_viz_shape[0].attrib[self.new_attr_name] = link_viz_shape[0].attrib.pop(old_attribute_name)
                        link_viz_shape[0].attrib.__delitem__('length')
                        if self.new_tag == 'mesh':
                            link_viz_shape[0].set(self.new_attr_name, self.link_viz_mesh)
                        elif self.new_tag == 'box':
                            link_viz_shape[0].set(self.new_attr_name, self.link_viz_box)
                        elif self.new_tag == 'sphere':
                            link_viz_shape[0].set(self.new_attr_name, self.link_viz_sph_r)

                link_col_origin = self.element.find(self.xacro_tag + 'link_col_origin')
                link_col_origin.set('xyz', self.link_col_xyz)

                link_col_origin.set('rpy', self.link_col_rpy)

                link_col_shape = self.element.find(self.xacro_tag + 'link_col_shape')
                self.shape_type_col = link_col_shape[0].tag
                if self.shape_type_col != 'cylinder':
                    if self.new_tag_col == 'cylinder':
                        link_col_shape[0].tag = self.new_tag_col
                        old_attribute_name = list(link_col_shape[0].attrib.keys())[0]
                        link_col_shape[0].attrib[self.new_attr_name_r_col] = link_col_shape[0].attrib.pop(old_attribute_name)
                        link_col_shape[0].set(self.new_attr_name_r_col, self.link_col_cyl_r)
                        link_col_shape[0].set(self.new_attr_name_l_col, self.link_col_cyl_l)
                    else:
                        link_col_shape[0].tag = self.new_tag_col
                        old_attribute_name = list(link_col_shape[0].attrib.keys())[0]
                        link_col_shape[0].attrib[self.new_attr_name_col] = link_col_shape[0].attrib.pop(old_attribute_name)
                        if self.new_tag_col == 'mesh':
                            link_col_shape[0].set(self.new_attr_name_col, self.link_col_mesh)
                        elif self.new_tag_col == 'box':
                            link_col_shape[0].set(self.new_attr_name_col, self.link_col_box)
                        elif self.new_tag_col == 'sphere':
                            link_col_shape[0].set(self.new_attr_name_col, self.link_col_sph_r)
                else:
                    if self.new_tag_col == 'cylinder':
                        link_col_shape[0].set(self.new_attr_name_r_col, self.link_col_cyl_r)
                        link_col_shape[0].set(self.new_attr_name_l_col, self.link_col_cyl_l)
                    else:
                        link_col_shape[0].tag = self.new_tag_col
                        old_attribute_name = list(link_col_shape[0].attrib.keys())[0]
                        link_col_shape[0].attrib[self.new_attr_name_col] = link_col_shape[0].attrib.pop(old_attribute_name)
                        link_col_shape[0].attrib.__delitem__('length')
                        if self.new_tag_col == 'mesh':
                            link_col_shape[0].set(self.new_attr_name_col, self.link_col_mesh)
                        elif self.new_tag_col == 'box':
                            link_col_shape[0].set(self.new_attr_name_col, self.link_col_box)
                        elif self.new_tag_col == 'sphere':
                            link_col_shape[0].set(self.new_attr_name_col, self.link_col_sph_r)

                link_color = self.element.find(self.xacro_tag + 'link_color')
                link_color.set('rgba', self.rgb)

                link_center_of_mass = self.element.find(self.xacro_tag + 'link_center_of_mass')
                link_center_of_mass.set('xyz', self.link_com)

                link_inertia = self.element.find(self.xacro_tag + 'link_inertia')
                link_inertia.set('ixx', self.link_xx)
                link_inertia.set('iyy', self.link_yy)
                link_inertia.set('izz', self.link_zz)
                link_inertia.set('ixy', self.link_xy)
                link_inertia.set('ixz', self.link_xz)
                link_inertia.set('iyz', self.link_yz)

                # Joint Elements
                if macro == 'part':
                    joint_origin = self.element.find(self.xacro_tag + 'joint_origin')
                    joint_origin.set('xyz', self.joint_xyz)
                    joint_origin.set('rpy', self.joint_rpy)

                    joint_axis_of_rotation = self.element.find(self.xacro_tag + 'joint_axis_of_rotation')
                    joint_axis_of_rotation.set('xyz', self.joint_aor)

                    joint_limits = self.element.find(self.xacro_tag + 'joint_limits')
                    joint_limits.set('effort', self.joint_effort)
                    joint_limits.set('velocity', self.joint_velocity)
                    joint_limits.set('lower', self.joint_lower)
                    joint_limits.set('upper', self.joint_upper)

                    joint_dynamics = self.element.find(self.xacro_tag + 'joint_dynamics')
                    joint_dynamics.set('damping', self.joint_damping)
                    print(self.joint_friction)
                    joint_dynamics.set('friction', self.joint_friction)
                    print(joint_dynamics.attrib)
                    
                    joint_mimic = self.element.find(self.xacro_tag + 'joint_mimic')
                    joint_mimic.set('joint', self.joint_mimic_joint)
                    joint_mimic.set('multiplier', self.joint_multiplier)
                    joint_mimic.set('offset', self.joint_offset)

                    if self.mimic == "1":
                        for element in self.root_ros2control.iter('joint'):
                            if element.attrib.get('name') == self.name + '_joint':

                                for sub_element in element.iter('param'):
                                    if sub_element.attrib.get('name') == 'mimic':
                                        sub_element.text = self.joint_mimic_joint + '_joint'
                                    if sub_element.attrib.get('name') == 'offset':
                                        sub_element.text = self.joint_offset
                                    if sub_element.attrib.get('name') == 'multiplier':
                                        sub_element.text = self.joint_multiplier

                                ET.register_namespace("xacro", 'http://www.ros.org/wiki/xacro')
                                ET.indent(self.tree_ros2control,'  ', level=0)
                                self.tree_ros2control.write(self.ros2_control_path)


    def update_camera_reference(self):

        name_to_change = False
        all_cameras = [self.ui.camera_select.itemText(i) for i in range(self.ui.camera_select.count()-1)]
        if not all_cameras:
            index = self.ui.camera_reference.findText(self.old_name)
            self.ui.camera_reference.setItemText(index, self.name)
        else:
            for camera in all_cameras:
                camera_select = camera.split()
                camera_reference = camera_select[2]
                if self.old_name == camera_reference.replace('_link',''): #TODO: Das ganze hätten wir nicht, wenn _link und joint nicht fest wären
                    name_to_change = True
                    if self.ui.camera_select.currentText() == camera:
                        self.ui.camera_select.setCurrentText('new camera')
                        self.ui.camera_select.setCurrentText(camera)
                    else:
                        self.ui.camera_select.setCurrentText(camera)

            if name_to_change == False:
                index = self.ui.camera_reference.findText(self.old_name)
                self.ui.camera_reference.setItemText(index, self.name)                

    def update_lidar_reference(self):

        name_to_change = False
        all_lidars = [self.ui.lidar_select.itemText(i) for i in range(self.ui.lidar_select.count()-1)]
        if not all_lidars:
            index = self.ui.lidar_reference.findText(self.old_name)
            self.ui.lidar_reference.setItemText(index, self.name)
        else:
            for lidar in all_lidars:
                lidar_select = lidar.split()
                lidar_reference = lidar_select[2]
                if self.old_name == lidar_reference.replace('_link',''):
                    if self.ui.lidar_select.currentText() == lidar:
                        self.ui.lidar_select.setCurrentText('new lidar')
                        self.ui.lidar_select.setCurrentText(lidar)
                    else:
                        self.ui.lidar_select.setCurrentText(lidar)

            if name_to_change == False:
                index = self.ui.lidar_reference.findText(self.old_name)
                self.ui.lidar_reference.setItemText(index, self.name)        

    def update_imu_reference(self):

        name_to_change = False
        all_imus = [self.ui.imu_select.itemText(i) for i in range(self.ui.imu_select.count()-1)]
        if not all_imus:
            index = self.ui.imu_reference.findText(self.old_name)
            self.ui.imu_reference.setItemText(index, self.name)
        else:
            for imu in all_imus:
                imu_select = imu.split()
                imu_reference = imu_select[2]
                if self.old_name == imu_reference.replace('_link',''):
                    if self.ui.imu_select.currentText() == imu:
                        self.ui.imu_select.setCurrentText('new imu')
                        self.ui.imu_select.setCurrentText(imu)
                    else:
                        self.ui.imu_select.setCurrentText(imu)

            if name_to_change == False:
                index = self.ui.imu_reference.findText(self.old_name)
                self.ui.imu_reference.setItemText(index, self.name)        

    def update_force_torque_reference(self):

        name_to_change = False
        all_fts = [self.ui.ft_select.itemText(i) for i in range(self.ui.ft_select.count()-1)]
        if not all_fts:
            index = self.ui.ft_reference.findText(self.old_name)
            self.ui.ft_reference.setItemText(index, self.name)
        else:
            for ft in all_fts:
                ft_select = ft.split()
                ft_reference = ft_select[2]
                if self.old_name == ft_reference.replace('_joint',''):
                    if self.ui.ft_select.currentText() == ft:
                        self.ui.ft_select.setCurrentText('new ft')
                        self.ui.ft_select.setCurrentText(ft)
                    else:
                        self.ui.ft_select.setCurrentText(ft)

            if name_to_change == False:
                index = self.ui.ft_reference.findText(self.old_name)
                self.ui.ft_reference.setItemText(index, self.name)        

    def update_ros2_control(self):  

        for element in self.root_ros2control.iter('param'):
            if element.attrib.get('name') == 'mimic' and element.text == self.old_name + '_joint':
                element.text = self.name + '_joint'
                ET.register_namespace("xacro", 'http://www.ros.org/wiki/xacro')
                ET.indent(self.tree_ros2control,'  ', level=0)
                self.tree_ros2control.write(self.ros2_control_path)

        index = self.ui.comboBox_ros2control_existing.findText(self.old_name)
        if index != -1:
            self.ui.comboBox_ros2control_existing.insertItem(index, self.name)
            self.ui.comboBox_ros2control_existing.setCurrentText(self.name)
            index = self.ui.comboBox_ros2control_existing.findText(self.old_name)
            self.ui.comboBox_ros2control_existing.removeItem(index)

        index = self.ui.comboBox_ros2control_missing.findText(self.old_name)
        if index != -1:
            self.ui.comboBox_ros2control_missing.insertItem(index, self.name)
            #self.ui.comboBox_ros2control_missing.setCurrentText(self.name)
            index = self.ui.comboBox_ros2control_missing.findText(self.old_name)
            self.ui.comboBox_ros2control_missing.removeItem(index)